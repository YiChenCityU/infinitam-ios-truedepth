#include "ITMViewBuilder_CPU.h"

#include "../../DeviceAgnostic/ITMViewBuilder.h"
#include "../../../../ORUtils/MetalContext.h"

using namespace ITMLib::Engine;
using namespace ORUtils;

ITMViewBuilder_CPU::ITMViewBuilder_CPU(const ITMRGBDCalib *calib):ITMViewBuilder(calib) { }
ITMViewBuilder_CPU::~ITMViewBuilder_CPU(void) { }

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise)
{ 
	printf("Calling UpdateView with rawDepthImage and modelSensorNoise\n");
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMView(calib, rgbImage->noDims, rawDepthImage->noDims, false);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(rawDepthImage->noDims, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(rawDepthImage->noDims, true, false);

		if (modelSensorNoise)
		{
			(*view_ptr)->depthNormal = new ITMFloat4Image(rawDepthImage->noDims, true, false);
			(*view_ptr)->depthUncertainty = new ITMFloatImage(rawDepthImage->noDims, true, false);
		}


	}
	ITMView *view = *view_ptr;

	view->rgb->SetFrom(rgbImage, MemoryBlock<Vector4u>::CPU_TO_CPU);
	this->shortImage->SetFrom(rawDepthImage, MemoryBlock<short>::CPU_TO_CPU);

	switch (view->calib->disparityCalib.type)
	{
    case ITMDisparityCalib::TRAFO_IPHONE:
        this->ConvertFP16ToFP32(view->depth, this->shortImage, view->calib->disparityCalib.params);
        break;
	case ITMDisparityCalib::TRAFO_KINECT:
		this->ConvertDisparityToDepth(view->depth, this->shortImage, &(view->calib->intrinsics_d), view->calib->disparityCalib.params);
		break;
	case ITMDisparityCalib::TRAFO_AFFINE:
        this->ConvertDepthAffineToFloat(view->depth, this->shortImage, view->calib->disparityCalib.params);
		break;
	default:
		break;
	}

	// Limit depth range to 40cm
	float *depthData = view->depth->GetData(MEMORYDEVICE_CPU);
	// Vector2i imgSize = view->depth->noDims;
	// for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
	// {
	// 	int locId = x + y * imgSize.x;
	// 	if (depthData[locId] > 0.4f) depthData[locId] = -1.0f;
	// }

	if (useBilateralFilter)
	{
		//5 steps of bilateral filtering
		this->DepthFiltering(this->floatImage, view->depth);
		this->DepthFiltering(view->depth, this->floatImage);
		this->DepthFiltering(this->floatImage, view->depth);
		this->DepthFiltering(view->depth, this->floatImage);
		this->DepthFiltering(this->floatImage, view->depth);
		view->depth->SetFrom(this->floatImage, MemoryBlock<float>::CPU_TO_CPU);
	}

	if (modelSensorNoise)
	{
		this->ComputeNormalAndWeights(view->depthNormal, view->depthUncertainty, view->depth, view->calib->intrinsics_d.projectionParamsSimple.all);
	}
}

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage)
{
	printf("Calling UpdateView with depthImage\n");
	if (*view_ptr == NULL)
		*view_ptr = new ITMView(calib, rgbImage->noDims, depthImage->noDims, false);

	ITMView *view = *view_ptr;

	view->rgb->UpdateDeviceFromHost();
	view->depth->UpdateDeviceFromHost();
}

void ITMViewBuilder_CPU::UpdateView(ITMView **view_ptr, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement)
{
	printf("Calling UpdateView with depthImage and IMU\n");
	if (*view_ptr == NULL)
	{
		*view_ptr = new ITMViewIMU(calib, rgbImage->noDims, depthImage->noDims, false);
		if (this->shortImage != NULL) delete this->shortImage;
		this->shortImage = new ITMShortImage(depthImage->noDims, true, false);
		if (this->floatImage != NULL) delete this->floatImage;
		this->floatImage = new ITMFloatImage(depthImage->noDims, true, false);
	}

	ITMViewIMU* imuView = (ITMViewIMU*)(*view_ptr);
	imuView->imu->SetFrom(imuMeasurement);

	this->UpdateView(view_ptr, rgbImage, depthImage, useBilateralFilter);
}

void ITMViewBuilder_CPU::ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics,
	Vector2f disparityCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	float fx_depth = depthIntrinsics->projectionParamsSimple.fx;

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDisparityToDepth(d_out, x, y, d_in, disparityCalibParams, fx_depth, imgSize);
}

void ITMViewBuilder_CPU::ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const Vector2f depthCalibParams)
{
	Vector2i imgSize = depth_in->noDims;

	const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
	float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
		convertDepthAffineToFloat(d_out, x, y, d_in, imgSize, depthCalibParams);
}

void ITMViewBuilder_CPU::ConvertFP16ToFP32(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const Vector2f depthCalibParams)
{
    Vector2i imgSize = depth_in->noDims;
    printf("Converting depth image of size: %d x %d\n", imgSize.x, imgSize.y);
    
    const short *d_in = depth_in->GetData(MEMORYDEVICE_CPU);
    float *d_out = depth_out->GetData(MEMORYDEVICE_CPU);
    
    for (int y = 0; y < imgSize.y; y++) for (int x = 0; x < imgSize.x; x++)
    {
        int locId = x + y * imgSize.x;
        short depth_in = d_in[locId];
        
        // 将 short 转换回 FP16
        __fp16 depth_fp16 = *((__fp16*)&depth_in);
        float depth = depth_fp16;
        
        // 不应用校准参数，直接使用原始深度值
        d_out[locId] = (depth > 0) ? depth : -1.0f;
        
        // 打印中心像素值用于调试
        if (x == imgSize.x/2 && y == imgSize.y/2) {
            printf("Center pixel:\n");
            printf("  Raw FP16 value: %d\n", depth_in);
            printf("  Converted to float: %f\n", depth);
            printf("  Final depth: %f\n", d_out[locId]);
        }
    }
}

void ITMViewBuilder_CPU::DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in)
{
	Vector2i imgSize = image_in->noDims;

	image_out->Clear();

	float *imout = image_out->GetData(MEMORYDEVICE_CPU);
	const float *imin = image_in->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgSize.y - 2; y++) for (int x = 2; x < imgSize.x - 2; x++)
		filterDepth(imout, imin, x, y, imgSize);
}

void ITMLib::Engine::ITMViewBuilder_CPU::ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic)
{
	Vector2i imgDims = depth_in->noDims;

	const float *depthData_in = depth_in->GetData(MEMORYDEVICE_CPU);

	float *sigmaZData_out = sigmaZ_out->GetData(MEMORYDEVICE_CPU);
	Vector4f *normalData_out = normal_out->GetData(MEMORYDEVICE_CPU);

	for (int y = 2; y < imgDims.y - 2; y++) for (int x = 2; x < imgDims.x - 2; x++)
		computeNormalAndWeight(depthData_in, normalData_out, sigmaZData_out, x, y, imgDims, intrinsic);
}
