/* This is the application file. */

#pragma once

#ifndef __MULTI_RGBD_CALIBRATION_APP_H__
#define __MULTI_RGBD_CALIBRATION_APP_H__

#include "MultiRGBDCalibrationConfig.h"
#include "MultiRGBDCalibrationUtil.h"
#include "RGBDCamera.h"

class MultiRGBDCalibrationApp
{
public:
	MultiRGBDCalibrationApp();
	virtual ~MultiRGBDCalibrationApp();

	void clear();
	void init();

	// Load the program configuration
	bool loadConfig(const std::string& fn);

	bool startMainLoop();

private:
	void _checkAppStatus();
	void _loadData();
	void _calibrate();
	void _saveResults();
	
	MultiRGBDCalibrationConfig m_config;

	/* ----- Application Status----- */
	bool m_bConfigLoaded;
	std::vector<bool> m_bCalibrateIntrinsicEnabled;

	/* ----- Data ----- */
	int m_numCamera;
	int m_numFrame;
	std::vector<CameraIntrinsicF>	m_intrinsics; // m_intrinsics[camId]
	std::vector<RGBDCamera>			m_rgbdCamera; // m_rgbdCamera[camId]


};

#endif//__MULTI_RGBD_CALIBRATION_APP_H__