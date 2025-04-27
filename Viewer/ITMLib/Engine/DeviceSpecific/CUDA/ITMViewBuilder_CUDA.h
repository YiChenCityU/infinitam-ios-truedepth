// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../../ITMViewBuilder.h"

namespace ITMLib
{
	namespace Engine
	{
		class ITMViewBuilder_CUDA : public ITMViewBuilder
		{
		public:
			void ConvertDisparityToDepth(ITMFloatImage *depth_out, const ITMShortImage *depth_in, const ITMIntrinsics *depthIntrinsics, 
				Vector2f disparityCalibParams);
			void ConvertDepthAffineToFloat(ITMFloatImage *depth_out, const ITMShortImage *depth_in, Vector2f depthCalibParams);

			void DepthFiltering(ITMFloatImage *image_out, const ITMFloatImage *image_in);
			void ComputeNormalAndWeights(ITMFloat4Image *normal_out, ITMFloatImage *sigmaZ_out, const ITMFloatImage *depth_in, Vector4f intrinsic);

			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage, bool useBilateralFilter, bool modelSensorNoise = false);
			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMFloatImage *depthImage);

			void UpdateView(ITMView **view, ITMUChar4Image *rgbImage, ITMShortImage *depthImage, bool useBilateralFilter, ITMIMUMeasurement *imuMeasurement);

			ITMViewBuilder_CUDA(const ITMRGBDCalib *calib);
			~ITMViewBuilder_CUDA(void);

			__global__ void convertDisparityToDepth_device(float *depth_out, const short *depth_in, Vector2f disparityCalibParams, float fx_depth, Vector2i imgSize);
			__global__ void convertDepthAffineToFloat_device(float *d_out, const short *d_in, Vector2i imgSize, Vector2f depthCalibParams);
			__global__ void filterDepth_device(float *imageData_out, const float *imageData_in, Vector2i imgDims);
			__global__ void ComputeNormalAndWeight_device(const float* depth_in, Vector4f* normal_out, float *sigmaL_out, Vector2i imgDims, Vector4f intrinsic);
			__global__ void limitDepthRange_device(float *depthData, Vector2i imgSize);
		};
	}
}
