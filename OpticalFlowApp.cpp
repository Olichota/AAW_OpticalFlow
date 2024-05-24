#include "OpticalFlowApp.hpp"

int OpticalFlowApp::readInputImages(std::string inputImageName1, std::string inputImageName2)
{	
	// load input bitmap image

    std::cout << inputImageName1 << std::endl;
    std::cout << inputImageName2 << std::endl;

    inputBitmap1.load(inputImageName1.c_str());
    inputBitmap2.load(inputImageName2.c_str());

    // error if image did not load
    if(!inputBitmap1.isLoaded() || !inputBitmap2.isLoaded())
    {
        std::cout << inputBitmap1.isLoaded() << std::endl;
        std::cout << inputBitmap2.isLoaded() << std::endl;

        std::cout << "Failed to load input images!" << std::endl;
        return SDK_FAILURE;
    }

    // get width and height of input image
    height1 = inputBitmap1.getHeight();
    width1 = inputBitmap1.getWidth();

    height2 = inputBitmap2.getHeight();
    width2 = inputBitmap2.getWidth();

    // allocate memory for input image data to host
	inputImage2DFirst = (cl_uchar4*)malloc(width1 * height1 * sizeof(cl_uchar4));
    CHECK_ALLOCATION(inputImage2DFirst,"Failed to allocate memory for first image! (inputImage2D)");
	
    inputImage2DSecond = (cl_uchar4*)malloc(width2 * height2 * sizeof(cl_uchar4));
    CHECK_ALLOCATION(inputImage2DSecond,"Failed to allocate memory for second image! (inputImage2D)");

	// get the pointer to pixel data
    pixelData1 = inputBitmap1.getPixels();
    pixelData2 = inputBitmap2.getPixels();
    if(pixelData1 == NULL || pixelData2 == NULL)
    {
        std::cout << "Failed to read pixel Data!";
        return SDK_FAILURE;
    }

    // Copy pixel data into inputImageData2D
    memcpy(inputImage2DFirst, pixelData1, width1 * height1 * pixelSize);
    memcpy(inputImage2DSecond, pixelData2, width2 * height2 * pixelSize);

	// allocate memory for output image data for Non-Separable Filter to host
    opticalFlowOutput = (cl_uchar4*)malloc(width1 * height1 * sizeof(cl_uchar4));
    CHECK_ALLOCATION(opticalFlowOutput,"Failed to allocate memory! (opticalFlowOutput)");
	memset(opticalFlowOutput, 0, width1 * height1 * pixelSize);

	// set local work-group size
	localThreads[0] = blockSizeX; 
	localThreads[1] = blockSizeY;

	// // set global work-group size
	globalThreads[0] = (width1 + localThreads[0] - 1) / localThreads[0];
    globalThreads[0] *= localThreads[0];
    globalThreads[1] = (height1 + localThreads[1] - 1) / localThreads[1];
    globalThreads[1] *= localThreads[1];

    return SDK_SUCCESS;
}

int OpticalFlowApp::writeOutputImage(std::string outputImageName, cl_uchar4 *outputImageData)
{
    // copy output image data back to original pixel data
    memcpy(pixelData1, outputImageData, width1 * height1 * pixelSize);

    // write the output bmp file
    if(!inputBitmap1.write(outputImageName.c_str()))
    {
        std::cout << "Failed to write output image!";
        return SDK_FAILURE;
    }

    return SDK_SUCCESS;
}

int OpticalFlowApp::setupCL(void)
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

    inputBuffer1 = clCreateBuffer(
                      context,
                      CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
					  pixelSize * width1 * height1,
					  inputImage2DFirst,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (inputBuffer1)");

    inputBuffer2 = clCreateBuffer(
                      context,
                      CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
					  pixelSize * width1 * height1,
					  inputImage2DSecond,
                      &status);
    CHECK_OPENCL_ERROR(status, "clCreateBuffer failed. (inputBuffer2)");

	outputBuffer = clCreateBuffer(
                       context,
                       CL_MEM_WRITE_ONLY,
					   pixelSize * width1 * height1,
                       NULL,
                       &status);
    CHECK_OPENCL_ERROR( status,  "clCreateBuffer failed. (outputBuffer)");

    // create a CL program using the kernel source
	char option[256];
    sprintf(option, "-DLOCAL_XRES=%d -DLOCAL_YRES=%d -DUSE_LDS=%d",
                    LOCAL_XRES, LOCAL_YRES, useLDSPass1);

    buildProgramData buildData;
    buildData.kernelName = std::string("OpticalFlowApp_Kernels.cl");
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

    // get a kernel object handle for a Non-Separable Filter
    opticalFlowKernel = clCreateKernel(program, "opticalFlowKernel", &status);
    CHECK_OPENCL_ERROR(status, "clCreateKernel failed (opticalFlowKernel).");

    return SDK_SUCCESS;
}

int OpticalFlowApp::runOpticalFlow(void)
{
    cl_int   status;
    cl_event events[1];

    // Set appropriate arguments to the kernel
    status = clSetKernelArg(
                 opticalFlowKernel,
                 0,
                 sizeof(cl_mem),
                 (void *)&inputBuffer1);
    CHECK_OPENCL_ERROR( status, "clSetKernelArg failed. (inputBuffer1)");

    status = clSetKernelArg(
                 opticalFlowKernel,
                 1,
                 sizeof(cl_mem),
                 (void *)&inputBuffer2);
    CHECK_OPENCL_ERROR( status, "clSetKernelArg failed. (inputBuffer2)");

	status = clSetKernelArg(
                 opticalFlowKernel,
                 2,
                 sizeof(cl_mem),
                 (void *)&outputBuffer);
    CHECK_OPENCL_ERROR( status, "clSetKernelArg failed. (outputBuffer)");

    status = clSetKernelArg(
                 opticalFlowKernel,
                 3,
                 sizeof(cl_uint),
				 (void *)&width1);
    CHECK_OPENCL_ERROR( status, "clSetKernelArg failed. (width1)");

    status = clSetKernelArg(
                 opticalFlowKernel,
                 4,
                 sizeof(cl_uint),
				 (void *)&height1);
    CHECK_OPENCL_ERROR( status, "clSetKernelArg failed. (height1)");

    // Enqueue a kernel run call.
    status = clEnqueueNDRangeKernel(
                 commandQueue,
                 opticalFlowKernel,
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

int OpticalFlowApp::initialize(int argc, char* argv[])
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

    // Check if there are enough command line arguments
    //if (argc != 3)
    //{
     //   std::cerr << "Usage: " << argv[0] << " <imageFile1> <imageFile2>" << std::endl;
    //    return SDK_FAILURE;
    //}

    // Load input images from command line arguments
    imageFile1 = argv[1];
    imageFile2 = argv[2];

    return SDK_SUCCESS;
}

cl_int OpticalFlowApp::loadInputImages()
{
    // Wczytaj ścieżki do dwóch obrazów wejściowych z argumentów wiersza poleceń
    // std::string imageFile1 = "C:\\Users\\licho\\Desktop\AAW\\LAB\\LAB_03_HelloWorld\\AdvancedConvolution\\I.bmp";
    // std::string imageFile2 = "C:\\Users\\licho\\Desktop\AAW\\LAB\\LAB_03_HelloWorld\\AdvancedConvolution\\J.bmp";
    //std::string imageFile1 = "D:\\Studia\\8_semestr\\AAW-GPU\\Proj\\AAW_OpticalFlow\\I.bmp";
    //std::string imageFile2 = "D:\\Studia\\8_semestr\\AAW-GPU\Proj\\AAW_OpticalFlow\\J.bmp";

    std::string imageFile1 = getPath() + "I.bmp";
    std::string imageFile2 = getPath() + "J.bmp";

    // Sprawdź czy pliki obrazów istnieją i wczytaj je
    if (!imageFile1.empty() && !imageFile2.empty()) 
    {
        if (readInputImages(imageFile1, imageFile2) != SDK_SUCCESS) 
        {
            std::cerr << "Could not load input images." << std::endl;
            return SDK_FAILURE;
        }
    } 
    else 
    {
        std::cerr << "Lack of input images." << std::endl;
        return SDK_FAILURE;
    }

    return SDK_SUCCESS;
}

int OpticalFlowApp::setup()
{
    // Allocate host memory and read input image
    // std::string filePath1 = getPath() + std::string(INPUT_IMAGE_OPTiCAL_FLOW);
    // std::string filePath2 = getPath() + std::string(OUTPUT_IMAGE_OPTiCAL_FLOW);
	int status = loadInputImages();
    // std::string filePath = getPath() + std::string(INPUT_IMAGE);
    // int status = readInputImage(filePath);
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


int OpticalFlowApp::run()
{
	int status;

    // Warm up
    for(int i = 0; i < 2 && iterations != 1; i++)
    {
		// run non-separable implementation of convolution
		if (runOpticalFlow() != SDK_SUCCESS)
		{
		     return SDK_FAILURE;
		}

		// Enqueue readBuffer for non-separable filter
		status = clEnqueueReadBuffer(
						commandQueue,
						outputBuffer,
						CL_TRUE,
						0,
						width1 * height1 * pixelSize,
						opticalFlowOutput,
						0,
						NULL,
						NULL);
		CHECK_OPENCL_ERROR( status, "clEnqueueReadBuffer(opticalFlowOutput) failed.");
    }

	std::cout << "Executing kernel for " << iterations <<
              " iterations" <<std::endl;
    std::cout << "-------------------------------------------" << std::endl;

	// create and initialize timers
    int timer = sampleTimer->createTimer();
	sampleTimer->resetTimer(timer);
    sampleTimer->startTimer(timer);

	// running non-separable filter
	for(int i = 0; i < iterations; i++)
    {
        status = runOpticalFlow();
        CHECK_ERROR(status, SDK_SUCCESS, "OpenCL run Kernel failed for Separable Filter");
    }

    sampleTimer->stopTimer(timer);
    totalOpticalFlowKernelTime = (double)(sampleTimer->readTimer(timer)) / iterations;

	// Enqueue readBuffer for non-separable filter
	status = clEnqueueReadBuffer(
					commandQueue,
					outputBuffer,
					CL_TRUE,
					0,
					width1 * height1 * pixelSize,
					opticalFlowOutput,
					0,
					NULL,
					NULL);
	CHECK_OPENCL_ERROR( status, "clEnqueueReadBuffer(opticalFlowOutput) failed.");

	// write the non-separable filter output image to bitmap file
    status = writeOutputImage(OUTPUT_IMAGE_OPTiCAL_FLOW, opticalFlowOutput);
    CHECK_ERROR(status, SDK_SUCCESS, "non-Separable Filter Output Image Failed");

    return SDK_SUCCESS;
}

void OpticalFlowApp::printStats()
{
    if(sampleArgs->timing)
    {
        std::string strArray[3] = {"Width", "Height", "KernelTime(sec)"};
        std::string stats[3];

		std::cout << "\n Optical Flow Timing Measurement!" << std::endl;
        stats[0] = toString(width1, std::dec);
        stats[1] = toString(height1, std::dec);
        stats[2] = toString(totalOpticalFlowKernelTime, std::dec);
        printStatistics(strArray, stats, 3);
    }
}

int OpticalFlowApp::cleanup()
{
    // Releases OpenCL resources (Context, Memory etc.)
    cl_int status;

	if (opticalFlowKernel != NULL)
	{
		status = clReleaseKernel(opticalFlowKernel);
		CHECK_OPENCL_ERROR(status, "clReleaseKernel failed.(nonSeparablekernel)");
	}

	if (program)
	{
		status = clReleaseProgram(program);
		CHECK_OPENCL_ERROR(status, "clReleaseProgram failed.(program)");
	}

    if (inputBuffer1) {
        status = clReleaseMemObject(inputBuffer1);
        CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(inputBuffer1)");
    }

    if (inputBuffer2) {
        status = clReleaseMemObject(inputBuffer2);
        CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(inputBuffer2)");
    }

	if (outputBuffer)
	{
		status = clReleaseMemObject(outputBuffer);
		CHECK_OPENCL_ERROR(status, "clReleaseMemObject failed.(outputBuffer)");
	}

    // release program resources (input memory etc.)
	FREE(inputImage2DFirst);
	FREE(inputImage2DSecond);
	FREE(opticalFlowOutput);
    FREE(devices);

    return SDK_SUCCESS;
}

int main(int argc, char * argv[])
{
    OpticalFlowApp clOpticalFlowApp;

    if (clOpticalFlowApp.initialize(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if (clOpticalFlowApp.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

	int status = clOpticalFlowApp.setup();
    if (status != SDK_SUCCESS)
    {
		clOpticalFlowApp.cleanup();
        return status;
    }

    if (clOpticalFlowApp.run() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

    if (clOpticalFlowApp.cleanup() != SDK_SUCCESS)
    {
        return SDK_FAILURE;
    }

	clOpticalFlowApp.printStats();
    return SDK_SUCCESS;
}
