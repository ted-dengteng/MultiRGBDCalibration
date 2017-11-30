/* This class solves the extrinsic param. for a pair of rgbd cameras. */

#pragma once

#ifndef __RGBD_CAMERA_PAIR_EXTRINSIC_SOLVER_H__
#define __RGBD_CAMERA_PAIR_EXTRINSIC_SOLVER_H__

#include "RGBDCamera.h"

class RGBDCameraPairExtrinsicSolver
{
public:
	RGBDCameraPairExtrinsicSolver();
	virtual ~RGBDCameraPairExtrinsicSolver();

	void solveGlobalVisual(const int& patternWidth, 
		const int& patternHeight,
		const float& patternLength,
		std::vector<RGBDCamera*> rgbdCamera);
	void solveGlobalGeom(const int& patternWidth,
		const int& patternHeight,
		const float& patternLength,
		std::vector<RGBDCamera*> rgbdCamera);

private:
	void _extractCorners3d(const corner3d_t& objectPoints,
		const corner2d_t& cameraPoints,
		const cv::Mat& cameraMatrix,
		const cv::Mat& distCoeffs,
		corner3d_t& corner3dwrtCamCoord);
	void _solveExtrinsicSVD(const corner3d_t& point0,
		const corner3d_t& point1,
		cv::Mat M);

	// temp storage for extrinsic computation
	std::vector<corner3d_t> m_corners3d; // m_corner3d[camId][cornerId]

};


#endif//__RGBD_CAMERA_PAIR_EXTRINSIC_SOLVER_H__