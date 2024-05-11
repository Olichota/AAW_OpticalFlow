/**********************************************************************
Copyright �2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

�   Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
�   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/


#ifndef ADVANCED_CONVOLUTION_H_
#define ADVANCED_CONVOLUTION_H_

/**
 * Header Files
 */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "CLUtil.hpp"
#include "SDKBitMap.hpp"

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.1"

#define INPUT_IMAGE_OPTiCAL_FLOW_1 "OpticalFlowImage_Input1.bmp"
#define INPUT_IMAGE_OPTiCAL_FLOW_2 "OpticalFlowImage_Input2.bmp"
#define OUTPUT_IMAGE_OPTiCAL_FLOW "OpticalFlowImage_Output.bmp"

#define LOCAL_XRES 16
#define LOCAL_YRES 16

#define WIDTH 640
#define HEIGHT 480

using namespace appsdk;

/**
 * OpticalFlowApp
 * Class implements OpenCL OpticalFlowApp sample
 */

class OpticalFlowApp
{
    private: 
        cl_double    setupTime;          /**< Time for setting up OpenCL */
        cl_double    totalOpticalFlowKernelTime;

        cl_uint      width1;
        cl_uint      height1;
        cl_uint      width2;
        cl_uint      height2;   

        cl_uchar4*	 inputImage2DFirst;
        cl_uchar4*	 inputImage2DSecond;
        cl_uchar4*   opticalFlowOutput;
        cl_uint		 useLDSPass1;		 /**< A flag to indicate whether LDS uses is true or false */
        
        cl_context   context ;            /**< CL context */
        cl_device_id *devices ;           /**< CL device list */
        cl_mem       inputBuffer1 ;        /**< CL memory input buffer */
        cl_mem       inputBuffer2 ;
        cl_mem       outputBuffer ;		 /**< CL memory output buffer */
        cl_command_queue commandQueue ;   /**< CL command queue */
        cl_program   program ;            /**< CL program  */
        cl_kernel    opticalFlowKernel;

        SDKBitMap inputBitmap1;			 /**< Bitmap class object */
        SDKBitMap inputBitmap2;
        uchar4* pixelData1;
        uchar4* pixelData2;
        cl_uint pixelSize;               /**< Size of a pixel in BMP format> */
        size_t blockSizeX;               /**< Work-group size in x-direction */
        size_t blockSizeY;               /**< Work-group size in y-direction */
        size_t       globalThreads[2];   /**< global NDRange */
        size_t       localThreads[2];    /**< Local Work Group Size */
        int          iterations;         /**< Number of iterations to execute kernel */
        SDKDeviceInfo deviceInfo;        /**< Structure to store device information*/
        KernelWorkGroupInfo kernelInfo;  /**< Structure to store kernel related info */

        SDKTimer *sampleTimer;      /**< SDKTimer object */

        std::string imageFile1;
        std::string imageFile2;

    public:

        CLCommandArgs   *sampleArgs;   /**< CLCommand argument class */

        cl_int loadInputImages();
        int runOpticalFlow();

		/**
        * Read bitmap image and allocate host memory
        * @param inputImageName name of the input file
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        */
        int readInputImages(std::string inputImageName1, std::string inputImageName2);

        /**
        * Write to an image file
        * @param outputImageName name of the output file
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        */
        int writeOutputImage(std::string outputImageName, cl_uchar4 *outputImageData);

        /**
         * Constructor
         * Initialize member variables
         */
        OpticalFlowApp()
        {
            sampleArgs = new CLCommandArgs();
            sampleTimer = new SDKTimer();
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
			blockSizeX = 16;
            blockSizeY = 16;
			pixelSize = sizeof(uchar4);
            pixelData1 = NULL;
            pixelData2 = NULL;
            setupTime = 0;
            totalOpticalFlowKernelTime = 0;
            iterations = 1;

            context = NULL;            /**< CL context */
			devices = NULL;           /**< CL device list */
			inputBuffer1 = NULL;        /**< CL memory input buffer */
			inputBuffer2 = NULL;
            outputBuffer = NULL;		 /**< CL memory output buffer */
			commandQueue = NULL;   /**< CL command queue */
			program = NULL;            /**< CL program  */
            opticalFlowKernel = NULL;
        }

        /**
         * OpenCL related initialisations.
         * Set up Context, Device list, Command Queue, Memory buffers
         * Build CL kernel program executable
         * @return SDK_SUCCESS on success and SDK_FAILURE0 on failure
         */
        int setupCL();

        /**
         * Override from SDKSample. Print sample stats.
         */
        void printStats();

        /**
         * Override from SDKSample. Initialize
         * command line parser, add custom options
         * @return SDK_SUCCESS on success and SDK_FAILURE0 on failure
         */
        int initialize(int argc, char* argv[]);

        /**
         * Override from SDKSample, adjust width and height
         * of execution domain, perform all sample setup
         * @return SDK_SUCCESS on success and SDK_FAILURE0 on failure
         */
        int setup();

        /**
         * Override from SDKSample
         * Run OpenCL Separable and Non-Separable Filter
		 * Get kernel start and end time if timing is enabled
         * @return SDK_SUCCESS on success and SDK_FAILURE0 on failure
         */
        int run();

        /**
         * Override from SDKSample
         * Cleanup memory allocations
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int cleanup();
};

#endif
