/**********************************************************************
Copyright �2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

�	Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
�	Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/



/**
*******************************************************************************
*  @fn     opticalFlow
*  @brief  This is the algorithm for optical flow.
*
*  @param[in] input1          : Buffer containing first input image 
*  @param[in] input2          : Buffer containing secnd input image 
*  @param[out] output         : Buffer containing the output of optical flow
*  @param[in] nWidth          : Image width in pixels
*  @param[in] nHeight         : Image height in pixels
*  @param[in] windowSize      : Size od window in pixels
* 
*******************************************************************************
*/

__kernel
__attribute__((reqd_work_group_size(LOCAL_XRES, LOCAL_YRES, 1)))
void opticalFlow(__global uchar4* input1,
                                __global uchar4* input2,
                                __global uchar4* output,
                                uint nWidth,
								uint nHeight,
                                uint windowSize
){

    int x = get_global_id(0);
    int y = get_global_id(1);
    
    

    // int windowSize = 9;  // Rozmiar okna 5x5
    int halfWindow = windowSize / 2;

    
    if (x < (halfWindow + 1) || x >= nWidth - (halfWindow + 1) || y < (halfWindow + 1) || y >= nHeight - (halfWindow + 1))
        return;

    float A = 0.0f, B = 0.0f, C = 0.0f, D = 0.0f, E = 0.0f;


    for (int j = -halfWindow; j <= halfWindow; j++) {
        for (int i = -halfWindow; i <= halfWindow; i++) {
            int currentIdx = (y + j) * nWidth + (x + i);
            
            float I1 = dot(convert_float4(input1[currentIdx]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
            float I2 = dot(convert_float4(input2[currentIdx]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));

            // Obliczanie gradientów przestrzennych i czasowych
            float Ix = (dot(convert_float4(input1[currentIdx + 1]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f)) -
                        dot(convert_float4(input1[currentIdx - 1]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f))) * 0.5f;
            float Iy = (dot(convert_float4(input1[currentIdx + nWidth]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f)) -
                        dot(convert_float4(input1[currentIdx - nWidth]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f))) * 0.5f;
            float It = I2 - I1;

            // Tensor struktury
            A += Ix * Ix;
            B += Iy * Iy;
            C += Ix * Iy;
            D += Ix * It;
            E += Iy * It;
        }
    }

    float u, v;

    float det = A * B - C * C;
    if (fabs(det) > 1e-6) { // uniknięcie dzielenia przez zero
        u = (E * C - D * B) / det;
        v = (D * C - A * E) / det;
    } else {
        u = 0.0f;
        v = 0.0f;
    }

    float angle = atan2(v, u);
    float magnitude = hypot(u, v);

    float normalized_angle = (angle + (float)M_PI) / (2.0f * (float)M_PI);

    float V = 1.0f;
    float H = normalized_angle * 360.0f;
    
    float S = magnitude/(float)halfWindow;



    float C_ = V * S;
    float X = C_ * (1.0f - fabs(fmod(H / 60.0f, 2.0f) - 1.0f));
    float m = V - C_;
    float r, g, b;
    if (H >= 0 && H < 60) {
        r = C_, g = X, b = 0;
    } else if (H >= 60 && H < 120) {
        r = X, g = C_, b = 0;
    } else if (H >= 120 && H < 180) {
        r = 0, g = C_, b = X;
    } else if (H >= 180 && H < 240) {
        r = 0, g = X, b = C_;
    } else if (H >= 240 && H < 300) {
        r = X, g = 0, b = C_;
    } else {
        r = C_, g = 0, b = X;
    }
    r = (r + m) * 255;
    g = (g + m) * 255;
    b = (b + m) * 255;



    output[y * nWidth + x] = (uchar4){(uchar)r,(uchar)g,(uchar)b,255};
}