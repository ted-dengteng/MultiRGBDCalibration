#pragma once

#ifndef __MULTI_RGBD_CALIBRATION_UTIL_H__
#define __MULTI_RGBD_CALIBRATION_UTIL_H__

#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

template <typename T>
struct CameraIntrinsic
{
	// image size
	int w, h;
	// focal length
	T fx, fy;
	// principal point
	T cx, cy;
	// distortion coefficients
	T dist[5]; 

	bool load(const std::string& fn)
	{
		std::ifstream intrFile(fn, std::ios::in);
		if (intrFile.is_open())
		{
			intrFile >> w >> h;
			intrFile >> fx >> fy;
			intrFile >> cx >> cy;
			for (int i = 0; i < 4; i++)
				intrFile >> dist[i];
			intrFile.close();
			return true;
		}
		else
			return false;
	}

	bool save(const std::string& fn)
	{
		std::ifstream intrFile(fn, std::ios::in);
		if (intrFile.is_open())
		{
			intrFile << w << " " << h << std::endl;
			intrFile << fx << " " << fy << std::endl;
			intrFile << cx << " " << cy << std::endl;
			for (int i = 0; i < 4; i++)
				intrFile << dist[i] << " ";
			intrFile.close();
			return true;
		}
		else
			return false;
	}

	void copyFrom(const CameraIntrinsic* intr)
	{
		w = intr->w;
		h = intr->h;
		fx = intr->fx;
		fy = intr->fy;
		cx = intr->cx;
		cy = intr->cy;
		for (int i = 0; i < 5; i++)
			dist[i] = intr->dist[i];
	}

	void printParam()
	{
		printf("w=%d h=%d\n", w, h);
		printf("fx=%f, fy=%f\n", fx, fy);
		printf("cx=%f, cy=%f\n", cx, cy);
		printf("dist=[ ");
		for (int i = 0; i < 5; i++)
			printf(" %f", dist[i]);
		printf(" ]\n");
	}
};

typedef CameraIntrinsic<float> CameraIntrinsicF;

template <typename T>
struct CameraExtrinsic
{
	T Rotation[3][3];
	T Translation[3];
};

typedef CameraExtrinsic<float> CameraExtrinsicF;

#endif//__MULTI_RGBD_CALIBRATION_UTIL_H__