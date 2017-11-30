/* This class manages the data for each rgbd camera.*/

#pragma once

#ifndef __RGBD_CAMERA_H__
#define __RGBD_CAMERA_H__

#include "MultiRGBDCalibrationUtil.h"

#define DEPTH_SAMPLE_RANGE 1 // pixels
#define DEPTH_SIMILARITY_THRESHOLD 100 // mm

// for debug
#define DEBUG_SHOW_DETECTED_CORNERS 1

class RGBDCamera
{
public:
	typedef std::vector<cv::Point2f> corner2d_t;
	typedef std::vector<cv::Point3f> corner3d_t;

	RGBDCamera();
	virtual ~RGBDCamera();

	void clear();
	void init(const std::vector<std::string> colorFilenames,
		const std::vector<std::string> depthFilenames,
		const int& patternWidth,
		const int& patternHeight,
		const float& patternLength,
		const CameraIntrinsicF* intrinsic = NULL);

private:
	void _loadColor(const std::vector<std::string> colorFilenames);
	void _loadDepth(const std::vector<std::string> depthFilenames);
	void _extractCorners2dCheckerboard(const cv::Size patternSize);
	void _extractCorners3d();
	void _computeIntrinsic(const cv::Size patternSize, const float& patternLength);

	int m_numFrame;
	CameraIntrinsicF* m_intrinsic;

	// frameId
	std::vector<bool> m_bPatternDetected;
	std::vector<cv::Mat*> m_color;
	std::vector<cv::Mat*> m_depth;

	// frameId, cornerId
	std::vector<corner2d_t> m_corners2d;
	std::vector<corner3d_t> m_corners3d;
};

#endif//__RGBD_CAMERA_H__