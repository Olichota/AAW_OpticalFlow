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


#include "AdvancedConvolution.hpp"

int
AdvancedConvolution::readInputImage(std::string inputImageFirstName, std::string inputImageSecondName)
{
	// Check whether isLds is zero or one 
	if(useLDSPass1 != 0 && useLDSPass1 != 1)
	{
		std::cout << "isLds should be either 0 or 1" << std::endl;
		return SDK_EXPECTED_FAILURE;
	}

	// initialization of mask 
	// if(numOfImages != 3 && numOfImages != 5)
	// {
	// 	std::cout << "Filter Size should be either 3 or 5" << std::endl;
	// 	return SDK_EXPECTED_FAILURE;
	// }

    // if (windowSize !=0 && windowSize != 1 && windowSize !=2)
    // {
    //     std::cout << "Filter Type can only be 0, 1 or 2 for Sobel, Box and Gaussian filters respectively." << std::endl;
	// 	return SDK_EXPECTED_FAILURE;
    // }

    // switch (windowSize)
    // {
    // case 0: /* Sobel Filter */
    //     if(numOfImages == 3)
	//     {
	// 	    mask = SOBEL_FILTER_3x3;
	// 	    rowFilter = SOBEL_FILTER_3x3_pass1;
	// 	    colFilter = SOBEL_FILTER_3x3_pass2;
	//     }
	//     else
	//     {
	// 	    mask = SOBEL_FILTER_5x5;
	// 	    rowFilter = SOBEL_FILTER_5x5_pass1;
	// 	    colFilter = SOBEL_FILTER_5x5_pass2;
	//     }
    //     break;

    // case 1: /* Box Filter */
    //     if(numOfImages == 3)
	//     {
	// 	    mask = BOX_FILTER_3x3;
	// 	    rowFilter = BOX_FILTER_3x3_pass1;
	// 	    colFilter = BOX_FILTER_3x3_pass2;
	//     }
	//     else
	//     {
	// 	    mask = BOX_FILTER_5x5;
	// 	    rowFilter = BOX_FILTER_5x5_pass1;
	// 	    colFilter = BOX_FILTER_5x5_pass2;
	//     }
    //     break;

    // case 2: /* Gaussian Filter */
    //     if(numOfImages == 3)
	//     {
	// 	    mask = GAUSSIAN_FILTER_3x3;
	// 	    rowFilter = GAUSSIAN_FILTER_3x3_pass1;
	// 	    colFilter = GAUSSIAN_FILTER_3x3_pass2;
	//     }
	//     else
	//     {
	// 	    mask = GAUSSIAN_FILTER_5x5;
	// 	    rowFilter = GAUSSIAN_FILTER_5x5_pass1;
	// 	    colFilter = GAUSSIAN_FILTER_5x5_pass2;
	//     }
    //     break;
    // }
	

	// load input bitmap image
    inputBitmapFirst.load(inputImageFirstName.c_str());
    inputBitmapSecond.load(inputImageSecondName.c_str());

    // error if image did not load
    if(!inputBitmapFirst.isLoaded())
    {
        std::cout << "Failed to load first input image!";
        return SDK_FAILURE;
    }

    if(!inputBitmapSecond.isLoaded())
    {
        std::cout << "Failed to load second input image!";
        return SDK_FAILURE;
    }



    // get width and height of input image
    height = inputBitmapFirst.getHeight();
    width = inputBitmapFirst.getWidth();

    // allocate memory for input image data to host
	inputFirstImage2D = (cl_uchar4*)malloc(width * height * sizeof(cl_uchar4));
    CHECK_ALLOCATION(inputFirstImage2D,"Failed to allocate memory! (inputFirstImage2D)");

    inputSecondImage2D = (cl_uchar4*)malloc(width * height * sizeof(cl_uchar4));
    CHECK_ALLOCATION(inputSecondImage2D,"Failed to allocate memory! (inputSecondImage2D)");


	
	// get the pointer to pixel data
    pixelFirstData = inputBitmapFirst.getPixels();
    if(pixelFirstData == NULL)
    {
        std::cout << "Failed to read first pixel Data!";
        return SDK_FAILURE;
    }

    pixelSecondData = inputBitmapSecond.getPixels();
    if(pixelSecondData == NULL)
    {
        std::cout << "Failed to read second pixel Data!";
        return SDK_FAILURE;
    }

    // Copy pixel data into inputImageData2D
    memcpy(inputFirstImage2D, pixelFirstData, width * height * pixelSize);
    memcpy(inputSecondImage2D, pixelSecondData, width * height * pixelSize);

	// allocate and initalize memory for padded input image data to host

	// filterRadius = numOfImages - 1;
	// paddedHeight = height + filterRadius;
	// paddedWidth = width + filterRadius;

    // paddedInputFirstImage2D = (cl_uchar4*)malloc(paddedWidth * paddedHeight * sizeof(cl_uchar4));
    // CHECK_ALLOCATION(paddedInputFirstImage2D,"Failed to allocate memory! (paddedInputFirstImage2D)");
	// memset(paddedInputFirstImage2D, 0, paddedHeight*paddedWidth*sizeof(cl_uchar4));
	// for(cl_uint i = filterRadius; i < height + filterRadius; i++)
	// {
	// 	for(cl_uint j = filterRadius; j < width + filterRadius; j++)
	// 	{
	// 		paddedInputFirstImage2D[i * paddedWidth + j] = inputFirstImage2D[(i - filterRadius) * width + (j - filterRadius)];		
	// 	}
	// }

	
    opticalFlowOutputImage2D = (cl_uchar4*)malloc(width * height * sizeof(cl_uchar4));
    CHECK_ALLOCATION(opticalFlowOutputImage2D,"Failed to allocate memory! (opticalFlowOutputImage2D)");
	memset(opticalFlowOutputImage2D, 0, width * height * pixelSize);

    opticalFlowVerificationOutput = (cl_uchar*)malloc(width * height * pixelSize);
    CHECK_ALLOCATION(opticalFlowVerificationOutput,"Failed to allocate memory! (verificationOutput)");

    memset(opticalFlowVerificationOutput, 0, width * height * pixelSize);

	// set local work-group size
	localThreads[0] = blockSizeX; 
	localThreads[1] = blockSizeY;

	// set global work-group size, padding work-items do not need to be considered
	globalThreads[0] = (width + localThreads[0] - 1) / localThreads[0];
    globalThreads[0] *= localThreads[0];
    globalThreads[1] = (height + localThreads[1] - 1) / localThreads[1];
    globalThreads[1] *= localThreads[1];

    return SDK_SUCCESS;
}

int
AdvancedConvolution::writeOutputImage(std::string outputImageName, cl_uchar4 *outputImageData)
{
    // copy output image data back to original pixel data
    memcpy(pixelFirstData, outputImageData, width * height * pixelSize);

    // write the output bmp file
    if(!inputBitmapFirst.write(outputImageName.c_str()))
    {
        std::cout << "Failed to write output image!";
        return SDK_FAILURE;
    }

    return SDK_SUCCESS;
}

// int
// AdvancedConvolution::genBinaryImage()
// {
//     bifData binaryData;
//     binaryData.kernelName = std::string("AdvancedConvolution_Kernels.cl");
//     binaryData.flagsStr = std::string("");
//     if(sampleArgs->isComplierFlagsSpecified())
//     {
//         binaryData.flagsFileName = std::string(sampleArgs->flags.c_str());
//     }

//     binaryData.binaryName = std::string(sampleArgs->dumpBinary.c_str());
//     int status = generateBinaryImage(binaryData);
//     return status;
// }


int
AdvancedConvolution::setupCL(void)
{
    cl_int status = 0;
    cl_device_type dType;

    if(sampleArgs->deviceType.compare("cpu") == 0)
    {
        dType = CL_DEVICE_TYPE_CPU;
    }
    else //deviceType = "gpu"
    {
        dType = CL_DEVICE_TYPE_GPU;
        if(sampleArgs->isThereGPU() == false)
        {
            std::cout << "GPU not found. Falling back to CPU device" << std::endl;
            dType = CL_DEVICE_TYPE_CPU;
        }
    }

    /*
     * Have a look at the available platforms and pick either
     * the AMD one if available or a reasonable default.
     */
    cl_platform_id platform = NULL;
    int retValue = getPlatform(platform, sampleArgs->platformId,
                               sampleArgs->isPlatformEnabled());
    CHECK_ERROR(retValue, SDK_SUCCESS, "getPlatform() failed");

    // Display available devices.
    retValue = displayDevices(platform, dType);
    CHECK_ERROR(retValue, SDK_SUCCESS, "displayDevices() failed");


    // If we could find our platform, use it. Otherwise use just available platform.
    cl_context_properties cps[3] =
    {
        CL_CONTEXT_PLATFORM,
        (cl_context_properties)platform,
        0
    };

    context = clCreateContextFromType(
                  cps,
                  dType,
                  NULL,
                  NULL,
                  &status);

    CHECK_OPENCL_ERROR( status, "clCreateContextFromType failed.");

    // getting device on which to run the sample
    status = getDevices(context, &devices, sampleArgs->deviceId,
                        sampleArgs->isDeviceIdEnabled());
    CHECK_ERROR(status, SDK_SUCCESS, "getDevices() failed");

	retValue = deviceInfo.setDeviceInfo(devices[sampleArgs->deviceId]);
	CHECK_ERROR(retValue, SDK_SUCCESS, "SDKDeviceInfo::setDeviceInfo() failed");

	if (localThreads[0] > deviceInfo.maxWorkItemSizes[0] ||
		localThreads[1] > deviceInfo.maxWorkItemSizes[1] ||
		(localThreads[0] * localThreads[1]) > deviceInfo.maxWorkGroupSize)
	{
		std::cout << "Unsupported: Device does not support requested"
			<< ":number of work items.";
		return SDK_EXPECTED_FAILURE;
	}

    {
        // The block is to move the declaration of prop closer to its use
        cl_command_queue_properties prop = 0;
        commandQueue = clCreateCommandQueue(
                           context,
                           devices[sampleArgs->deviceId],
                           prop,
                           &status);
        CHECK_OPENCL_ERROR( status, "clCreateCommandQueue failed.");
    }

    inputFirstBuffer = clCreateBuffer(
                      context,
                      CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
					  pixelSize * width * height,
					  inputFirstImage2D,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (inputFirstBuffer)");

    inputSecondBuffer = clCreateBuffer(
                      context,
                      CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
					  pixelSize * width * height,
					  inputSecondImage2D,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (inputSecondBuffer)");


	outputBuffer = clCreateBuffer(
                       context,
                       CL_MEM_WRITE_ONLY,
					   pixelSize * width * height,
                       NULL,
                       &status);
    CHECK_OPENCL_ERROR( status,  "clCreateBuffer failed. (outputBuffer)");

	// maskBuffer = clCreateBuffer(
    //                  context,
    //                  CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
	// 				 sizeof(cl_float ) * numOfImages * numOfImages,
    //                  mask,
    //                  &status);
    // CHECK_OPENCL_ERROR( status, "clCreateBuffer failed. (maskBuffer)");

	// rowFilterBuffer = clCreateBuffer(
    //                  context,
    //                  CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
    //                  sizeof(cl_float ) * numOfImages,
    //                  rowFilter,
    //                  &status);
    // CHECK_OPENCL_ERROR( status, "clCreateBuffer failed. (rowFilterBuffer)");

	// colFilterBuffer = clCreateBuffer(
    //                  context,
    //                  CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
    //                  sizeof(cl_float ) * numOfImages,
    //                  colFilter,
    //                  &status);
    // CHECK_OPENCL_ERROR( status, "clCreateBuffer failed. (colFilterBuffer)");

    // create a CL program using the kernel source
	char option[256];
    sprintf(option, "-DFILTERSIZE=%d -DLOCAL_XRES=%d -DLOCAL_YRES=%d -DUSE_LDS=%d",
                    numOfImages, LOCAL_XRES, LOCAL_YRES, useLDSPass1);

    buildProgramData buildData;
    buildData.kernelName = std::string("AdvancedConvolution_Kernels.cl");
    buildData.devices = devices;
    buildData.deviceId = sampleArgs->deviceId;
    buildData.flagsStr = std::string(option);
    if(sampleArgs->isLoadBinaryEnabled())
    {
        buildData.binaryName = std::string(sampleArgs->loadBinary.c_str());
    }

    if(sampleArgs->isComplierFlagsSpecified())
    {
        buildData.flagsFileName = std::string(sampleArgs->flags.c_str());
    }

    retValue = buildOpenCLProgram(program, context, buildData);
    CHECK_ERROR(retValue, SDK_SUCCESS, "buildOpenCLProgram() failed");

    // get a kernel object handle for a Optical Flow
    opticalFlowkernel = clCreateKernel(program, "opticalFlow", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel failed (opticalFlow).");

    return SDK_SUCCESS;
}

int
AdvancedConvolution::runOpticalFlowCLKernels(void)
{
    cl_int   status;
    cl_event events[1];

    // Set appropriate arguments to the kernel
    status = clSetKernelArg(
                 opticalFlowkernel,
                 0,
                 sizeof(cl_mem),
                 (void *)&inputFirstBuffer);
    CHECK_OPENCL_ERROR( status, "clSetKernelArg failed. (inputFirstBuffer)");

    status = clSetKernelArg(
                 opticalFlowkernel,
                 1,
                 sizeof(cl_mem),
                 (void *)&inputSecondBuffer);
    CHECK_OPENCL_ERROR( status, "clSetKernelArg failed. (inputSecondBuffer)");

    status = clSetKernelArg(
                 opticalFlowkernel,
                 2,
                 sizeof(cl_mem),
                 (void *)&outputBuffer);
    CHECK_OPENCL_ERROR( status, "clSetKernelArg failed. (outputBuffer)");	

    status = clSetKernelArg(
                 opticalFlowkernel,
                 3,
                 sizeof(cl_uint),
				 (void *)&width);
    CHECK_OPENCL_ERROR( status, "clSetKernelArg failed. (width)");

    status = clSetKernelArg(
                 opticalFlowkernel,
                 4,
                 sizeof(cl_uint),
				 (void *)&height);
    CHECK_OPENCL_ERROR( status, "clSetKernelArg failed. (height)");

    status = clSetKernelArg(
                 opticalFlowkernel,
                 5,
                 sizeof(cl_uint),
				 (void *)&windowSize);
    CHECK_OPENCL_ERROR( status, "clSetKernelArg failed. (windowSize)");

    // Enqueue a kernel run call.
    status = clEnqueueNDRangeKernel(
                 commandQueue,
                 opticalFlowkernel,
                 2,
                 NULL,
                 globalThreads,
                 localThreads,
                 0,
                 NULL,
                 &events[0]);
    CHECK_OPENCL_ERROR( status, "clEnqueueNDRangeKernel failed.");

    status = clFlush(commandQueue);
    CHECK_OPENCL_ERROR(status,"clFlush() failed");

    status = waitForEventAndRelease(&events[0]);
    CHECK_ERROR(status, SDK_SUCCESS, "WaitForEventAndRelease(events[0]) Failed");
    

    return SDK_SUCCESS;
}


/**
 * Reference CPU implementation of Advanced Convolution kernel
 * for performance comparison
 */
// void
// AdvancedConvolution::CPUReference()
// {	
//     for(cl_uint i = 0; i < height; ++i)
//     {
//         for(cl_uint j = 0; j < width; ++j)
//         {
// 			cl_float4 sum = {0.0f, 0.0f, 0.0f, 0.0f};
// 			for(cl_uint m = 0; m < numOfImages; m++)
// 			{
// 				for(cl_uint n = 0; n < numOfImages; n++)
// 				{
// 					cl_uint maskIndex = m*numOfImages+n;
// 					cl_uint inputIndex = (i+m)*width + (j+n);

// 					// copy uchar4 data to float4
// 					sum.s[0] += (cl_float)(inputFirstImage2D[inputIndex].s[0]) * (mask[maskIndex]);
// 					sum.s[1] += (cl_float)(inputFirstImage2D[inputIndex].s[1]) * (mask[maskIndex]);
// 					sum.s[2] += (cl_float)(inputFirstImage2D[inputIndex].s[2]) * (mask[maskIndex]);
// 					sum.s[3] += (cl_float)(inputFirstImage2D[inputIndex].s[3]) * (mask[maskIndex]);
// 				}
// 			}

// 			// calculating cpu reference for advanced convolution kernel
// 			nonSepVerificationOutput[((i*width + j) * 4) + 0] = (cl_uchar)((sum.s[0] < 0) ? 0 : ((sum.s[0] > 255.0) ? 255 : sum.s[0]));
// 			nonSepVerificationOutput[((i*width + j) * 4) + 1] = (cl_uchar)((sum.s[1] < 0) ? 0 : ((sum.s[1] > 255.0) ? 255 : sum.s[1]));
// 			nonSepVerificationOutput[((i*width + j) * 4) + 2] = (cl_uchar)((sum.s[2] < 0) ? 0 : ((sum.s[2] > 255.0) ? 255 : sum.s[2]));
// 			nonSepVerificationOutput[((i*width + j) * 4) + 3] = (cl_uchar)((sum.s[3] < 0) ? 0 : ((sum.s[3] > 255.0) ? 255 : sum.s[3]));
// 		}
// 	}	
// }

int AdvancedConvolution::initialize()
{
    // Call base class Initialize to get default configuration
    if  (sampleArgs->initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // Now add customized options
	Option* num_iterations = new Option;
    CHECK_ALLOCATION(num_iterations, "Memory allocation error.\n");
    num_iterations->_sVersion = "i";
    num_iterations->_lVersion = "iterations";
    num_iterations->_description = "Number of iterations for kernel execution";
    num_iterations->_type = CA_ARG_INT;
    num_iterations->_value = &iterations;
    sampleArgs->AddOption(num_iterations);
    delete num_iterations;

	Option* use_lds = new Option;
    CHECK_ALLOCATION(use_lds, "Memory allocation error.\n");
	useLDSPass1 = 1;
    use_lds->_sVersion = "l";
    use_lds->_lVersion = "useLDSPass1";
    use_lds->_description = "Use LDS for Pass1 of Separable Filter";
    use_lds->_type = CA_ARG_INT;
    use_lds->_value = &useLDSPass1;
    sampleArgs->AddOption(use_lds);
    delete use_lds;

    Option* mask_width = new Option;
    CHECK_ALLOCATION(mask_width, "Memory allocation error.\n");
	numOfImages = 2;
    mask_width->_sVersion = "m";
    mask_width->_lVersion = "Number of Images";
    mask_width->_description = "Number of images for optical flow - Supported values: 2 or more";
    mask_width->_type = CA_ARG_INT;
    mask_width->_value = &numOfImages;
    sampleArgs->AddOption(mask_width);
    delete mask_width;

    Option* filter_type = new Option;
    CHECK_ALLOCATION(filter_type, "Memory allocation error.\n");
	windowSize = 9;
    filter_type->_sVersion = "f";
    filter_type->_lVersion = "Window size";
    filter_type->_description = "Window size - size of window to compute optical flow";
    filter_type->_type = CA_ARG_INT;
    filter_type->_value = &windowSize;
    sampleArgs->AddOption(filter_type);
    delete filter_type;

    return SDK_SUCCESS;
}

int AdvancedConvolution::setup()
{
    // Allocate host memory and read input image
	std::string firstFilePath = getPath() + std::string(INPUT_IMAGE_FIRST);
    std::string secondFilePath = getPath() + std::string(INPUT_IMAGE_SECOND);
    int status = readInputImage(firstFilePath, secondFilePath);
    CHECK_ERROR(status, SDK_SUCCESS, "Read Input Image failed");

	// create and initialize timers
    int timer = sampleTimer->createTimer();
    sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    status = setupCL();
    if(status != SDK_SUCCESS)
    {
        return status;
    }

    sampleTimer->stopTimer(timer);

    setupTime = (cl_double)sampleTimer->readTimer(timer);

    return SDK_SUCCESS;
}


int AdvancedConvolution::run()
{
	int status;

    // Warm up
    for(int i = 0; i < 2 && iterations != 1; i++)
    {   // run opticalFlow implementation of convolution
		if (runOpticalFlowCLKernels() != SDK_SUCCESS)
		{
		     return SDK_FAILURE;
		}

		// Enqueue readBuffer for optical flow
		status = clEnqueueReadBuffer(
						commandQueue,
						outputBuffer,
						CL_TRUE,
						0,
						width * height * pixelSize,
						opticalFlowOutputImage2D,
						0,
						NULL,
						NULL);
		CHECK_OPENCL_ERROR( status, "clEnqueueReadBuffer(opticalFlowOutputImage2D) failed.");
    }

	std::cout << "Executing kernel for " << iterations <<
              " iterations" <<std::endl;
    std::cout << "-------------------------------------------" << std::endl;

	// create and initialize timers
    int timer = sampleTimer->createTimer();
	sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

    std::cout << "AAAAAAAa" << std::endl;

    // running optical flow
	for(int i = 0; i < iterations; i++)
    {
        status = runOpticalFlowCLKernels();
        CHECK_ERROR(status, SDK_SUCCESS, "OpenCL run Kernel failed for Optical Flow");
    }

    std::cout << "AAAAAAAa" << std::endl;

    sampleTimer->stopTimer(timer);
    totalOpticalFlowKernelTime = (double)(sampleTimer->readTimer(timer)) / iterations;

	// Enqueue readBuffer for optical flow
	status = clEnqueueReadBuffer(
					commandQueue,
					outputBuffer,
					CL_TRUE,
					0,
					width * height * pixelSize,
					opticalFlowOutputImage2D,
					0,
					NULL,
					NULL);
	CHECK_OPENCL_ERROR( status, "clEnqueueReadBuffer(opticalFlowOutputImage2D) failed.");

    std::cout << "AAAAAAAa" << std::endl;

	// write the optical flow output image to bitmap file
    status = writeOutputImage(OUTPUT_IMAGE_OPTICAL_FLOW, opticalFlowOutputImage2D);
    CHECK_ERROR(status, SDK_SUCCESS, "optical flow Output Image Failed");

    return SDK_SUCCESS;
}

// int AdvancedConvolution::verifyResults()
// {
//     if(sampleArgs->verify)
//     {
//         /**
// 		* Reference implementation on host device
// 		*/
// 		CPUReference();

// 		float *outputDevice = new float[width * height * pixelSize];
//         CHECK_ALLOCATION(outputDevice,"Failed to allocate host memory! (outputDevice)");
	
// 		float *outputReference = new float[width * height * pixelSize];
// 		CHECK_ALLOCATION(outputReference, "Failed to allocate host memory! (outputReference)");

// 		std::cout << "Verifying advanced non-Separable Convolution Kernel result - ";

// 		for(int i = 0; i < (int)(width * height); i++)
// 		{
// 			// copy uchar data to float array from verificationConvolutionOutput
// 			outputReference[i * 4 + 0] = nonSepVerificationOutput[i * 4 + 0];
// 			outputReference[i * 4 + 1] = nonSepVerificationOutput[i * 4 + 1];
// 			outputReference[i * 4 + 2] = nonSepVerificationOutput[i * 4 + 2];
// 			outputReference[i * 4 + 3] = nonSepVerificationOutput[i * 4 + 3];

// 			// copy uchar data to float array from global kernel
// 			outputDevice[i * 4 + 0] = nonSepOutputImage2D[i].s[0];
// 			outputDevice[i * 4 + 1] = nonSepOutputImage2D[i].s[1];
// 			outputDevice[i * 4 + 2] = nonSepOutputImage2D[i].s[2];
// 			outputDevice[i * 4 + 3] = nonSepOutputImage2D[i].s[3];
// 		}		

// 		// compare the results and see if they match
//         if(compare(outputDevice, outputReference, width * height * 4))
//         {
//             std::cout << "Passed!\n" << std::endl;
//         }
//         else
//         {
// 			delete[] outputDevice;
// 			delete[] outputReference;
//             std::cout << "Failed\n" << std::endl;
//             return SDK_FAILURE;
//         }

// 		std::cout << "Verifying advanced Separable Convolution Kernel result - ";

// 		memset(outputDevice, 0, width*height*4);
//         for(int i = 0; i < (int)(width * height); i++)
//         {
// 			// copy uchar data to float array from global kernel
//             outputDevice[i * 4 + 0] = sepOutputImage2D[i].s[0];
//             outputDevice[i * 4 + 1] = sepOutputImage2D[i].s[1];
//             outputDevice[i * 4 + 2] = sepOutputImage2D[i].s[2];
//             outputDevice[i * 4 + 3] = sepOutputImage2D[i].s[3];
//         }

//         // compare the results and see if they match
//         if(compare(outputDevice, outputReference, width * height * 4))
//         {
// 			delete[] outputDevice;
// 			delete[] outputReference;
//             std::cout << "Passed!\n" << std::endl;
//         }
//         else
//         {
// 			delete[] outputDevice;
// 			delete[] outputReference;
//             std::cout << "Failed!\n" << std::endl;
//             return SDK_FAILURE;
//         }
//     }

//     return SDK_SUCCESS;
// }

void AdvancedConvolution::printStats()
{
    if(sampleArgs->timing)
    {
        std::cout << "\n Optical Flow Timing Measurement!" << std::endl;
        std::string strArray[4] = {"Width", "Height", "Num of photos", "KernelTime(sec)"};
        std::string stats[4];
        stats[0] = toString(width, std::dec);
        stats[1] = toString(height, std::dec);
		stats[2] = toString(numOfImages, std::dec);
        stats[3] = toString(totalOpticalFlowKernelTime, std::dec);
        printStatistics(strArray, stats, 4);
    }
}

int AdvancedConvolution::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status;

    if (opticalFlowkernel != NULL)
	{
		status = clReleaseKernel(opticalFlowkernel);
		CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(opticalFlowkernel)");
	}

	if (program)
	{
		status = clReleaseProgram(program);
		CHECK_OPENCL_ERROR(status, "clReleaseProgram failed.(program)");
	}

	if (inputFirstBuffer)
	{
		status = clReleaseMemObject(inputFirstBuffer);
		CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(inputFirstBuffer)");
	}

    if (inputSecondBuffer)
	{
		status = clReleaseMemObject(inputSecondBuffer);
		CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(inputSecondBuffer)");
	}

	if (outputBuffer)
	{
		status = clReleaseMemObject(outputBuffer);
		CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(outputBuffer)");
	}

	// if (maskBuffer)
	// {
	// 	status = clReleaseMemObject(maskBuffer);
	// 	CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(maskBuffer)");
	// }

	// if (rowFilterBuffer)
	// {
	// 	status = clReleaseMemObject(rowFilterBuffer);
	// 	CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(rowFilterBuffer)");
	// }

	// if(colFilterBuffer)
	// {
	// 	status = clReleaseMemObject(colFilterBuffer);
	// 	CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(colFilterBuffer)");
	// }

	if (commandQueue)
	{
		status = clReleaseCommandQueue(commandQueue);
		CHECK_OPENCL_ERROR(status, "clReleaseCommandQueue failed.(commandQueue)");
	}

	if (context)
	{
		status = clReleaseContext(context);
		CHECK_OPENCL_ERROR(status, "clReleaseContext failed.(context)");
	}

    // release program resources (input memory etc.)
	FREE(inputFirstImage2D);
    FREE(inputSecondImage2D);
	// FREE(paddedInputFirstImage2D);
    FREE(opticalFlowOutputImage2D);
    FREE(opticalFlowVerificationOutput);
    FREE(devices);

    return SDK_SUCCESS;
}

int
main(int argc, char * argv[])
{
    AdvancedConvolution clAdvancedConvolution;

    if (clAdvancedConvolution.initialize() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if (clAdvancedConvolution.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // if(clAdvancedConvolution.sampleArgs->isDumpBinaryEnabled())
    // {
    //     return clAdvancedConvolution.genBinaryImage();
    // }

	int status = clAdvancedConvolution.setup();
    if (status != SDK_SUCCESS)
    {

		clAdvancedConvolution.cleanup();
        return status;
    }

    if (clAdvancedConvolution.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    // if (clAdvancedConvolution.verifyResults() != SDK_SUCCESS)
    // {
    //     return SDK_FAILURE;
    // }

    if (clAdvancedConvolution.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

	clAdvancedConvolution.printStats();
    return SDK_SUCCESS;
}
