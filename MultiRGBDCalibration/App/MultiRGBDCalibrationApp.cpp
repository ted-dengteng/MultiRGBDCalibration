#include "MultiRGBDCalibrationApp.h"

MultiRGBDCalibrationApp::MultiRGBDCalibrationApp() : m_bConfigLoaded(false), m_numCamera(0), m_numFrame(0)
{

}

MultiRGBDCalibrationApp::~MultiRGBDCalibrationApp()
{

}

void MultiRGBDCalibrationApp::clear()
{
	m_intrinsics.clear();

	m_numCamera = 0;
	m_numFrame = 0;
}

void MultiRGBDCalibrationApp::init()
{
	if (!m_bConfigLoaded)
	{
		printf("Config. not loaded! App. initilization failed!\n");
		exit(0);
	}

	m_numCamera = m_config.numCamera;
	m_numFrame = m_config.numFrame;

	m_intrinsics.resize(m_numCamera);
	m_bCalibrateIntrinsicEnabled.resize(m_numCamera);
}

bool MultiRGBDCalibrationApp::loadConfig(const std::string& fn)
{
	m_config.loadConfig(fn);
	m_bConfigLoaded = true;

	init();

	return true;
}

bool MultiRGBDCalibrationApp::startMainLoop()
{
	_checkAppStatus();
	_loadData();
	_calibrate();
	_saveResults();

	return true;
}

void MultiRGBDCalibrationApp::_checkAppStatus()
{
	if (!m_bConfigLoaded)
	{
		printf("Error! config. not loaded!\n");
		exit(0);
	}

	// check if need intrinsic calibration
	for (int camId = 0; camId < m_config.numCamera; camId++)
	{
		if (m_intrinsics[camId].load(m_config.initIntrinsicFilenames[camId]))
			m_bCalibrateIntrinsicEnabled[camId] = false;
		else 
			m_bCalibrateIntrinsicEnabled[camId] = true;
	}

}

void MultiRGBDCalibrationApp::_loadData()
{
	m_rgbdCamera.resize(m_numCamera);
	for (int camId = 0; camId < m_numCamera; camId++)
	{
		m_rgbdCamera[camId].init(m_config.colorFilenames[camId],
			m_config.depthFilenames[camId],
			m_config.patternWidth,
			m_config.patternHeight,
			m_config.patternLength);
	}
}

void MultiRGBDCalibrationApp::_calibrate()
{

}

void MultiRGBDCalibrationApp::_saveResults()
{

}