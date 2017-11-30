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
	RGBDCamera();
	virtual ~RGBDCamera();

	void clear();
	void init(const std::vector<std::string> colorFilenames,
		const std::vector<std::string> depthFilenames,
		const int& patternWidth,
		const int& patternHeight,
		const float& patternLength,
		const CameraIntrinsicF* intrinsic = NULL);

	const int getNumFrame() const
	{
		return m_numFrame;
	}
	int getNumFrame()
	{
		return m_numFrame;
	}

	const cv::Mat getCameraMatrix() const
	{
		return m_cameraMatrix;
	}
	cv::Mat getCameraMatrix()
	{
		return m_cameraMatrix;
	}
	const cv::Mat getDistCoeffs() const
	{
		return m_distCoeffs;
	}
	cv::Mat getDistCoeffs()
	{
		return m_distCoeffs;
	}

	corner2d_t& getCorner2d(int frameId)
	{
		return m_corners2d[frameId];
	}
	corner3d_t& getCorner3d(int frameId)
	{
		return m_corners3d[frameId];
	}


	bool isPatternDetected(int frameId)
	{
		if (frameId < 0 || frameId >= m_numFrame)
			return false;
		return m_bPatternDetected[frameId];
	}

private:
	void _loadColor(const std::vector<std::string> colorFilenames);
	void _loadDepth(const std::vector<std::string> depthFilenames);
	void _extractCorners2dCheckerboard(const cv::Size patternSize);
	void _extractCorners3d();
	void _computeIntrinsic(const cv::Size patternSize, const float& patternLength);

	int m_numFrame;
	CameraIntrinsicF* m_intrinsic;
	cv::Mat m_cameraMatrix;
	cv::Mat m_distCoeffs;

	// frameId
	std::vector<bool> m_bPatternDetected;
	std::vector<cv::Mat*> m_color;
	std::vector<cv::Mat*> m_depth;

	// frameId, cornerId
	std::vector<corner2d_t> m_corners2d;
	std::vector<corner3d_t> m_corners3d;
};

#endif//__RGBD_CAMERA_H__