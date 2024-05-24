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
* 
*******************************************************************************
*/

__constant sampler_t imageSampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP | CLK_FILTER_LINEAR; 

// __kernel void opticalFlowKernel(__read_only image2d_t input1,
//                                 __read_only image2d_t input2,
//                                 __write_only image2d_t output,
//                                 uint nWidth,
// 								uint nHeight

__kernel void opticalFlowKernel(__global uchar4* input1,
                                __global uchar4* input2,
                                __global uchar4* output,
                                uint nWidth,
								uint nHeight
){
    // uint x = get_global_id(0);
    // uint y = get_global_id(1);

    // float i1 = dot(convert_float4(input1[y*nWidth + x]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
    // float i1_u = dot(convert_float4(input1[(y-1)*nWidth + x]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
    // float i1_d = dot(convert_float4(input1[(y+1)*nWidth + x]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
    // float i1_l = dot(convert_float4(input1[y*nWidth + (x-1)]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
    // float i1_r = dot(convert_float4(input1[y*nWidth + (x+1)]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
    // float i2 = dot(convert_float4(input2[y*nWidth + x]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));

    // if (x > 0 && y > 0 && x < nWidth - 1 && y < nHeight - 1) {
    //     // Obliczanie gradientów
    //     float Ix = (float)(i1_r - i1_l) / 2.0f;
    //     float Iy = (float)(i1_d - i1_u) / 2.0f;
    //     float It = (float)(i2 - i1);

    //     // Macierz A i wektor b
    //     float A11 = Ix * Ix;
    //     float A12 = Ix * Iy;
    //     float A21 = A12;
    //     float A22 = Iy * Iy;
    //     float b1 = -Ix * It;
    //     float b2 = -Iy * It;

    //     // Obliczanie odwrotności macierzy A
    //     float det = A11 * A22 - A12 * A21;
    //     if (det != 0.0f) {
    //         float invDet = 1.0f / det;
    //         float invA11 = invDet * A22;
    //         float invA12 = -invDet * A12;
    //         float invA21 = -invDet * A21;
    //         float invA22 = invDet * A11;

    //         // Obliczanie przepływu optycznego u, v
    //         float u = invA11 * b1 + invA12 * b2;
    //         float v = invA21 * b1 + invA22 * b2;

    //         // Wizualizacja przepływu optycznego
    //         float color_scale = 20.0f; // Można dostosować skalę dla lepszej wizualizacji
    //         // output[(y * nWidth + x) * 3 + 0] = (uchar)fabs(u * color_scale);
    //         // output[(y * nWidth + x) * 3 + 1] = (uchar)fabs(v * color_scale);
    //         // output[(y * nWidth + x) * 3 + 2] = 0;
    //         // output[(y * nWidth + x) * 3 + 0] = 255;
    //         // output[(y * nWidth + x) * 3 + 1] = 255;
    //         // output[(y * nWidth + x) * 3 + 2] = 255;

    //         float angle = atan2(v, u);
    //         float magnitude = hypot(u, v);

    //         float normalized_angle = (angle + (float)M_PI) / (2.0f * (float)M_PI);

    //         float S = 1.0f, V = 1.0f;
    //         float H = normalized_angle * 360.0f;
            
    //         float C = V * S;
    //         float X = C * (1.0f - fabs(fmod(H / 60.0f, 2.0f) - 1.0f));
    //         float m = V - C;
    //         float r, g, b;
    //         if (H >= 0 && H < 60) {
    //             r = C, g = X, b = 0;
    //         } else if (H >= 60 && H < 120) {
    //             r = X, g = C, b = 0;
    //         } else if (H >= 120 && H < 180) {
    //             r = 0, g = C, b = X;
    //         } else if (H >= 180 && H < 240) {
    //             r = 0, g = X, b = C;
    //         } else if (H >= 240 && H < 300) {
    //             r = X, g = 0, b = C;
    //         } else {
    //             r = C, g = 0, b = X;
    //         }
    //         r = (r + m) * 255;
    //         g = (g + m) * 255;
    //         b = (b + m) * 255;



    //         output[(y * nWidth + x)] = (uchar4)(r,g,b,0);
    //     } else {
    //         // W przypadku, gdy det = 0, ustawiamy piksel na czarno
    //         // output[(y * nWidth + x) * 3 + 0] = 0;
    //         // output[(y * nWidth + x) * 3 + 1] = 0;
    //         // output[(y * nWidth + x) * 3 + 2] = 0;

    //         output[(y * nWidth + x)] = (uchar4)(0,0,0,0);
    //     }
    // }




    // int x = get_global_id(0);
    // int y = get_global_id(1);

    // int index = y * nWidth + x;

    // float I1 = dot(convert_float4(input1[index]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
    // float I2 = dot(convert_float4(input2[index]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));

    // float iy_1 = dot(convert_float4(input1[index+nWidth]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
    // float iy_2 = dot(convert_float4(input1[index-nWidth]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
    // float ix_1 = dot(convert_float4(input1[index+1]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
    // float ix_2 = dot(convert_float4(input1[index-1]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
    
    // float Ix = (ix_1 - ix_2) * 0.5f;
    // float Iy = (iy_1 - iy_2) * 0.5f;
    // float It = I2 - I1;

    // float A = Ix * Ix;
    // float B = Iy * Iy;
    // float C = Ix * Iy;
    // float D = Ix * It;
    // float E = Iy * It;

    // float det = A * B - C * C;

    // float u, v;

    // if (fabs(det) > 1e-6) { // uniknięcie dzielenia przez zero
    //     u = (E * C - D * B) / det;
    //     v = (D * C - A * E) / det;
    // } else {
    //     u = 0.0f;
    //     v = 0.0f;
    // }

    // float angle = atan2(v, u);
    // float magnitude = hypot(u, v);

    // float normalized_angle = (angle + (float)M_PI) / (2.0f * (float)M_PI);

    // float S = 1.0f, V = 1.0f;
    // float H = normalized_angle * 360.0f;
    
    // float C_ = V * S;
    // float X = C_ * (1.0f - fabs(fmod(H / 60.0f, 2.0f) - 1.0f));
    // float m = V - C_;
    // float r, g, b;
    // if (H >= 0 && H < 60) {
    //     r = C_, g = X, b = 0;
    // } else if (H >= 60 && H < 120) {
    //     r = X, g = C_, b = 0;
    // } else if (H >= 120 && H < 180) {
    //     r = 0, g = C_, b = X;
    // } else if (H >= 180 && H < 240) {
    //     r = 0, g = X, b = C_;
    // } else if (H >= 240 && H < 300) {
    //     r = X, g = 0, b = C_;
    // } else {
    //     r = C_, g = 0, b = X;
    // }
    // r = (r + m) * 255;
    // g = (g + m) * 255;
    // b = (b + m) * 255;

    // output[index] = (uchar4)(r,g,b,0);



    int x = get_global_id(0);
    int y = get_global_id(1);
    
    if (x < 2 || x >= nWidth - 2 || y < 2 || y >= nHeight - 2)
        return;

    int windowSize = 9;  // Rozmiar okna 5x5
    int halfWindow = windowSize / 2;

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

    float S = 1.0f, V = 1.0f;
    float H = normalized_angle * 360.0f;
    
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



    output[y * nWidth + x] = (uchar4)(r,g,b,255);













    // uint width = nWidth;
	// uint height = nHeight;

	// float4 Gx = (float4)(0);
	// float4 Gy = Gx;
	
	// int c = x + y * nWidth;


	// /* Read each texel component and calculate the filtered value using neighbouring texel components */
	// if( x >= 1 && x < (width-1) && y >= 1 && y < height - 1)
	// {
	// 	float4 i00 = convert_float4(input1[c - 1 - width]);
	// 	float4 i10 = convert_float4(input1[c - width]);
	// 	float4 i20 = convert_float4(input1[c + 1 - width]);
	// 	float4 i01 = convert_float4(input1[c - 1]);
	// 	float4 i11 = convert_float4(input1[c]);
	// 	float4 i21 = convert_float4(input1[c + 1]);
	// 	float4 i02 = convert_float4(input1[c - 1 + width]);
	// 	float4 i12 = convert_float4(input1[c + width]);
	// 	float4 i22 = convert_float4(input1[c + 1 + width]);

	// 	Gx =   i00 + (float4)(2) * i10 + i20 - i02  - (float4)(2) * i12 - i22;

	// 	Gy =   i00 - i20  + (float4)(2)*i01 - (float4)(2)*i21 + i02  -  i22;

	// 	/* taking root of sums of squares of Gx and Gy */
	// 	output[c] = convert_uchar(255);

	// }


    // int2 coord = (int2)(get_global_id(0), get_global_id(1));

	// float pixel = convert_float(read_imageui(input1, imageSampler, (int2)(coord.x, coord.y)).x);


    // write_imageui(output, coord, convert_uint(pixel));


    // int col = get_global_id(0);
    // int row = get_global_id(1);

    // printf("%d \n", col);

    // output[row*nWidth + col] = input1[row*nWidth + col];
    // float grey1 = dot(convert_float4(input1[row*nWidth + col]).xyz, (float3)(0.2989f, 0.5870f, 0.1140f));
    

}
