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
#include "FilterCoeff.h"

#define SAMPLE_VERSION "AMD-APP-SDK-v3.0.130.1"

#define INPUT_PREFIX "im"

#define LOCAL_XRES 16
#define LOCAL_YRES 16

using namespace appsdk;

/**
 * AdvancedConvolution
 * Class implements OpenCL AdvancedConvolution sample
 */

class AdvancedConvolution
{
        cl_double    setupTime;          /**< Time for setting up OpenCL */

        cl_uint      width;              /**< Width of the Input array */
        cl_uint      height;             /**< Height of the Input array */
		cl_uint		 paddedWidth;
		cl_uint      paddedHeight;

		cl_uchar4*	 inputFirstImage2D;
        cl_uchar4*	 inputSecondImage2D;
		cl_uchar4*	 paddedInputFirstImage2D;
        cl_uchar4*	 paddedInputSecondImage2D;

        cl_uchar4*   opticalFlowOutputImage2D;

        cl_uchar*	 opticalFlowVerificationOutput;

		cl_float     *mask;              /**< mask array */
		cl_float     *rowFilter;		 /**< Row-wise filter for pass1 */
		cl_float     *colFilter;		 /**< Column-wise filter for pass2 */


        cl_uint      windowSize;         /**< windowSize: 0: Sobel filter, 1: Box filter > */
		cl_uint		 filterRadius;		 /**< Filter Radius */
		cl_uint		 padding;			 /**< Padding Width */
		cl_uint		 useLDSPass1;		 /**< A flag to indicate whether LDS uses is true or false */
		
        cl_context   context ;            /**< CL context */
		cl_device_id *devices ;           /**< CL device list */
		cl_mem       inputFirstBuffer ;        /**< CL memory input buffer */
        cl_mem       inputSecondBuffer ;        /**< CL memory input buffer */
		cl_mem       outputBuffer ;		 /**< CL memory output buffer */
		cl_mem       maskBuffer ;         /**< CL memory mask buffer */
		cl_mem       rowFilterBuffer ;    /**< CL memory row filter buffer */
		cl_mem       colFilterBuffer ;    /**< CL memory col filter buffer */
		cl_command_queue commandQueue ;   /**< CL command queue */
		cl_program   program ;            /**< CL program  */
        cl_kernel    opticalFlowkernel ; /**< CL kernel for Non-Separable Filter */

		SDKBitMap inputBitmapFirst;	     /**< Bitmap class object */
        SDKBitMap inputBitmapSecond;	 /**< Bitmap class object */
		uchar4* pixelFirstData;			   	 /**< Pointer to image data */
        uchar4* pixelSecondData;			   	 /**< Pointer to image data */
        cl_uint pixelSize;               /**< Size of a pixel in BMP format> */
		size_t blockSizeX;               /**< Work-group size in x-direction */
        size_t blockSizeY;               /**< Work-group size in y-direction */
        size_t       globalThreads[2];   /**< global NDRange */
        size_t       localThreads[2];    /**< Local Work Group Size */
        int          iterations;         /**< Number of iterations to execute kernel */
        SDKDeviceInfo deviceInfo;        /**< Structure to store device information*/
        KernelWorkGroupInfo kernelInfo;  /**< Structure to store kernel related info */

        SDKTimer *sampleTimer;      /**< SDKTimer object */

    public:
        cl_uint		 numOfImages;		 /**< FilterSize */
        cl_double    opticalFlowKernelTime;	
        cl_double    totalOpticalFlowKernelTime;	
        CLCommandArgs   *sampleArgs;   /**< CLCommand argument class */
        std::string outputImageName;

		/**
        * Read bitmap image and allocate host memory
        * @param inputFirstImageName name of the input file
        * @param inputSecondImageName name of the input file
        * @return SDK_SUCCESS on success and SDK_FAILURE on failure
        */
        int readInputImage(std::string inputFirstImageName, std::string inputSecondImageName);

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
        AdvancedConvolution()
        {
            sampleArgs = new CLCommandArgs();
            sampleTimer = new SDKTimer();
            sampleArgs->sampleVerStr = SAMPLE_VERSION;
			blockSizeX = 16;
            blockSizeY = 16;
			pixelSize = sizeof(uchar4);
			pixelFirstData = NULL;
            setupTime = 0;
            opticalFlowKernelTime = 0;
            totalOpticalFlowKernelTime = 0;
            outputImageName = "OpticalFlow_1-2.bmp";
            iterations = 1;

			 context = NULL;            /**< CL context */
			devices = NULL;           /**< CL device list */
			inputFirstBuffer = NULL;        /**< CL memory input buffer */
            inputSecondBuffer = NULL;        /**< CL memory input buffer */
			outputBuffer = NULL;		 /**< CL memory output buffer */
			maskBuffer = NULL;         /**< CL memory mask buffer */
			rowFilterBuffer = NULL;    /**< CL memory row filter buffer */
			colFilterBuffer = NULL;    /**< CL memory col filter buffer */
			commandQueue = NULL;   /**< CL command queue */
			program = NULL;            /**< CL program  */
            opticalFlowkernel = NULL;
        }

        /**
         * Allocate and initialize host memory array for the given
		 * Input Image
         * @return SDK_SUCCESS on success and SDK_FAILURE0 on failure
         */
        int setupAdvancedConvolution();

        /**
         * OpenCL related initialisations.
         * Set up Context, Device list, Command Queue, Memory buffers
         * Build CL kernel program executable
         * @return SDK_SUCCESS on success and SDK_FAILURE0 on failure
         */
        int setupCL();

        /**
		* Call both non-separable and separable OpenCL implementation of 
		* Convolution
		* @return SDK_SUCCESS on success and SDK_FAILURE0 on failure
		*/
        int runCLKernels();

        /**
         * Set values for Non-Separable kernels' arguments, enqueue calls to the kernels
         * on to the command queue, wait till end of kernel execution.
         * @return SDK_SUCCESS on success and SDK_FAILURE0 on failure
         */
		int runOpticalFlowCLKernels();

        /**
         * Reference CPU implementation of Advanced Convolution
         * for performance comparison
         */
        void CPUReference();

        /**
         * Override from SDKSample. Print sample stats.
         */
        void printStats();

        /**
         * Override from SDKSample. Initialize
         * command line parser, add custom options
         * @return SDK_SUCCESS on success and SDK_FAILURE0 on failure
         */
        int initialize();

        /**
         * Override from SDKSample, Generate binary image of given kernel
         * and exit application
         * @return SDK_SUCCESS on success and SDK_FAILURE0 on failure
         */
        int genBinaryImage();

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

        /**
         * Override from SDKSample
         * Verify against reference implementation
         * @return SDK_SUCCESS on success and SDK_FAILURE on failure
         */
        int verifyResults();
};

#endif
