/* This file declares the configuration for the program. 
*
* The calibration folder is organized as follows:
*  ROOT_FOLDER/ 
*  ©À©¤©¤ (optional) config.ini
*  ©À©¤©¤ CalibrationParam/
*  ©¦   ©À©¤©¤ [CamXXName].intr
*  ©¦   ©À©¤©¤ ...  
*  ©¦   ©À©¤©¤ [CamXXName]_globalvis.extr
*  ©¦   ©À©¤©¤ ...
*  ©¦   ©À©¤©¤ [CamXXName]_globalgeom.extr
*  ©¦   ©À©¤©¤ ...
*  ©¦   ©À©¤©¤ [CamXXName]_local.extr
*  ©¦   ©¸©¤©¤ ...
*  ©¦
*  ©À©¤©¤ Depth-[CamXXName]/
*  ©¦   ©À©¤©¤ depth[XXXX].png
*  ©¦   ©¸©¤©¤ ...
*  ©À©¤©¤ (other Depth folders) ...
*  ©¦
*  ©À©¤©¤ Color-[CamXXName]/
*  ©¦   ©À©¤©¤ color[XXXX].png
*  ©¦   ©¸©¤©¤ ...
*  ©¸©¤©¤ (other Color folders)...
*
*/

#pragma once

#ifndef __MULTI_RGBD_CALIBRATION_CONFIG_H__
#define __MULTI_RGBD_CALIBRATION_CONFIG_H__

#include <string>
#include <vector>
#include <direct.h>

#include "..\Utility\dirent.h"
#include "..\Utility\INIReader.h"

struct MultiRGBDCalibrationConfig
{
public:
	enum EXTRINSIC_CALIB_METHOD{GLOBAL_VIS, GLOBAL_GEOM, LOCAL};

	// calibration method - 0 : global, 1 : local
	EXTRINSIC_CALIB_METHOD calibMethod;

	// path of configuration file
	std::string configFilename;

	/* ----- Calibration Sequences ----- */
	// num. of rgb-d cameras
	int numCamera;
	// num. of rgb-d frames used for calibration
	int numFrame;
	// name of rgb-d cameras
	std::vector<std::string> cameraName;

	/* ----- Checkerboard ----- */
	int patternWidth, patternHeight;
	float patternLength;

	/* ----- Local Rigid Volume ----- */
	int	resX, resY, resZ;
	float stX, stY, stZ;
	float edX, edY, edZ;

	/* ----- Folders ----- */
	std::string rootFolder;
	 
	// folder containing calibration parameters
	std::string paramFolder;

	// folders containing depth images
	std::vector<std::string> depthFolders; // depthFolders[camId]
	std::vector<std::vector<std::string>> depthFilenames; //depthFilenames[camId][frameId]

	// folders containing color images
	std::vector<std::string> colorFolders; // colorFolders[camId]
	std::vector<std::vector<std::string>> colorFilenames; // colorFilenames[camId][frameId]

	// rgbd calibration
	std::string intrinsicFolder;
	std::vector<std::string> initIntrinsicFilenames;
	std::vector<std::string> intrinsicFilenames;
	std::vector<std::string> extrinsicFilenames;

	int loadConfig(const std::string& fn)
	{
		char buffer[255];

		configFilename = fn;

		INIReader reader(fn);

		if (reader.ParseError() < 0)
		{
			printf("Couldn't load %s\n", fn.c_str());
			return 1;
		}

		/* ----- Path ----- */
		rootFolder	= reader.Get("input", "rootFolder", ".");
		sprintf_s(buffer, 255, "%s/CalibrationParam", rootFolder.c_str());
		paramFolder = std::string(buffer);
		_checkFolder(paramFolder);

		numFrame	= reader.GetInteger("input", "numFrame", -1);
		numCamera	= reader.GetInteger("input", "numCamera", -1);

		/* ----- Camera ----- */
		cameraName.resize(numCamera);
		depthFolders.resize(numCamera);
		colorFolders.resize(numCamera);
		depthFilenames.resize(numCamera);
		colorFilenames.resize(numCamera);
		initIntrinsicFilenames.resize(numCamera);
		intrinsicFilenames.resize(numCamera);
		extrinsicFilenames.resize(numCamera);
		for (int camId = 0; camId < numCamera; camId++)
		{
			sprintf_s(buffer, 255, "CamName%d", camId);
			cameraName[camId] = reader.Get("input", std::string(buffer), "");

			// folders
			sprintf_s(buffer, 255, "%s/Depth-%s", rootFolder.c_str(), cameraName[camId].c_str());
			depthFolders[camId] = std::string(buffer);
			_checkFolder(depthFolders[camId]);

			depthFilenames[camId].resize(numFrame);
			for (int frameId = 0; frameId < numFrame; frameId++)
			{
				sprintf_s(buffer, 255, "%s/depth%04d.png", depthFolders[camId].c_str(), frameId);
				depthFilenames[camId][frameId] = std::string(buffer);
			}

			sprintf_s(buffer, 255, "%s/Color-%s", rootFolder.c_str(), cameraName[camId].c_str());
			colorFolders[camId] = std::string(buffer);
			_checkFolder(colorFolders[camId]);

			colorFilenames[camId].resize(numFrame);
			for (int frameId = 0; frameId < numFrame; frameId++)
			{
				sprintf_s(buffer, 255, "%s/color%04d.png", colorFolders[camId].c_str(), frameId);
				colorFilenames[camId][frameId] = std::string(buffer);
			}

			// parameter paths
			sprintf_s(buffer, 255, "%s/%s_init.intr", paramFolder.c_str(), cameraName[camId].c_str());
			initIntrinsicFilenames[camId] = std::string(buffer);
			sprintf_s(buffer, 255, "%s/%s.intr", paramFolder.c_str(), cameraName[camId].c_str());
			intrinsicFilenames[camId] = std::string(buffer);
			sprintf_s(buffer, 255, "%s/%s_%s.extr", paramFolder.c_str(), cameraName[camId].c_str(), _getMethodName(calibMethod).c_str());
			extrinsicFilenames[camId] = std::string(buffer);
		}
		
		/* ----- Checkerboard ----- */
		patternWidth = reader.GetInteger("checkerboard", "width", -1);
		patternHeight = reader.GetInteger("checkerboard", "height", -1);
		patternLength = reader.GetReal("checkerboard", "length", -1.0);

		/* ----- Local Rigid Volume ----- */
		resX = reader.GetInteger("localvolume", "resolutionX", 12);
		resY = reader.GetInteger("localvolume", "resolutionY", 12);
		resZ = reader.GetInteger("localvolume", "resolutionZ", 10);

		stX = reader.GetReal("localvolume", "startX", -1.8);
		stY = reader.GetReal("localvolume", "startY", -1.8);
		stZ = reader.GetReal("localvolume", "startZ", 1.5);

		edX = reader.GetReal("localvolume", "endX", 1.8);
		edY = reader.GetReal("localvolume", "endY", 1.8);
		edZ = reader.GetReal("localvolume", "endZ", 4.5);

		return 0;
	}

	inline
		std::string _getMethodName(const EXTRINSIC_CALIB_METHOD calibMethod)
	{
		switch (calibMethod)
		{
		case GLOBAL_VIS:
			return "_globalvis";
		case GLOBAL_GEOM:
			return "_globalgeom";
		case LOCAL:
			return "_local";
		}
		return "";
	}

	inline
		void _checkFolder(const std::string& folder)
	{
		if (!opendir(folder.c_str()))
		{
			_mkdir(folder.c_str());
		}
	}

};


#endif//__MULTI_RGBD_CALIBRATION_CONFIG_H__