#include "RGBDCameraPairExtrinsicSolver.h"

RGBDCameraPairExtrinsicSolver::RGBDCameraPairExtrinsicSolver()
{

}

RGBDCameraPairExtrinsicSolver::~RGBDCameraPairExtrinsicSolver()
{

}

void RGBDCameraPairExtrinsicSolver::solveGlobalVisual(const int& patternWidth,
	const int& patternHeight,
	const float& patternLength,
	std::vector<RGBDCamera*> rgbdCamera)
{
	if (rgbdCamera.size() != 2)
	{
		printf("Error! The number of cameras should be two! Current num. = %d\n", rgbdCamera.size());
		return;
	}

	// get checkerboard corners
	corner3d_t corner3dRef;
	for (int j = 0; j < patternHeight; j++)
		for (int i = 0; i < patternWidth; i++)
			corner3dRef.push_back(cv::Point3d(float(j*patternLength), float(i*patternLength), 0));

	int numFrame = (int) rgbdCamera[0]->getNumFrame();
	int numCamera = (int) rgbdCamera.size();

	for (int camId = 0; camId < numCamera; camId++)
	{
		m_corners3d[camId].clear();
	}

	for (int frameId = 0; frameId < numFrame; frameId++)
	{
		// skip if checkerboard is not detected
		if (!rgbdCamera[0]->isPatternDetected(frameId)
			|| !rgbdCamera[0]->isPatternDetected(frameId))
			continue;

		for (int camId = 0; camId < numCamera; camId++)
		{
			corner3d_t tempCorner3dwrtCamCoord;
			// find points in 3d
			_extractCorners3d(corner3dRef,
				rgbdCamera[camId]->getCorner2d(frameId),
				rgbdCamera[camId]->getCameraMatrix(),
				rgbdCamera[camId]->getDistCoeffs(),
				tempCorner3dwrtCamCoord);

			// update
			for (int cornerId = 0; cornerId < tempCorner3dwrtCamCoord.size(); cornerId++)
				m_corners3d[camId].push_back(tempCorner3dwrtCamCoord[cornerId]);
		}
	}
}

void RGBDCameraPairExtrinsicSolver::solveGlobalGeom(const int& patternWidth,
	const int& patternHeight,
	const float& patternLength,
	std::vector<RGBDCamera*> rgbdCamera)
{
	if (rgbdCamera.size() != 2)
	{
		printf("Error! The number of cameras should be two! Current num. = %d\n", rgbdCamera.size());
		return;
	}

	int numFrame = (int)rgbdCamera[0]->getNumFrame();
	int numCamera = (int)rgbdCamera.size();

	for (int camId = 0; camId < numCamera; camId++)
	{
		m_corners3d[camId].clear();
	}

	for (int frameId = 0; frameId < numFrame; frameId++)
	{
		// skip if checkerboard is not detected
		if (!rgbdCamera[0]->isPatternDetected(frameId)
			|| !rgbdCamera[0]->isPatternDetected(frameId))
			continue;

		for (int camId = 0; camId < numCamera; camId++)
		{
			// update from depth
			for (int cornerId = 0; cornerId < rgbdCamera[camId]->getCorner3d(frameId).size(); cornerId++)
				m_corners3d[camId].push_back(rgbdCamera[camId]->getCorner3d(frameId)[cornerId]);
		}
	}
}

void RGBDCameraPairExtrinsicSolver::_extractCorners3d(const corner3d_t& objectPoints,
	const corner2d_t& cameraPoints,
	const cv::Mat& cameraMatrix,
	const cv::Mat& distCoeffs,
	corner3d_t& corner3dwrtCamCoord)
{
	cv::Mat rmat(3, 3, CV_64F);
	cv::Mat rvec(3, 1, CV_64F);
	cv::Mat tvec(3, 1, CV_64F);
	cv::Mat p_ref(3, 1, CV_64F);
	cv::Mat p_cam(3, 1, CV_64F);

	cv::solvePnP(objectPoints,
		cameraPoints,
		cameraMatrix,
		distCoeffs,
		rvec,
		tvec,
		false);

	cv::Rodrigues(rvec, rmat);

	const int numPoint = (int) objectPoints.size();
	corner3dwrtCamCoord.resize(numPoint);
	for (int cornerId = 0; cornerId < numPoint; cornerId++)
	{
		p_ref.at<double>(0) = objectPoints[cornerId].x;
		p_ref.at<double>(1) = objectPoints[cornerId].y;
		p_ref.at<double>(2) = objectPoints[cornerId].z;

		p_cam = rmat * p_ref + tvec;

		corner3dwrtCamCoord[cornerId].x = (float) p_cam.at<double>(0);
		corner3dwrtCamCoord[cornerId].y = (float) p_cam.at<double>(0);
		corner3dwrtCamCoord[cornerId].z = (float) p_cam.at<double>(0);
	}
}

void RGBDCameraPairExtrinsicSolver::_solveExtrinsicSVD(const corner3d_t& pointTarget,
	const corner3d_t& pointSource,
	cv::Mat M)
{
	int numCamera = 2;

	std::vector<corner3d_t> p(numCamera);
	p[0] = pointSource;
	p[1] = pointTarget;

	int numPoint = p[0].size();

	// compute mean and deduct mean
	std::vector<cv::Point3f> centroid(numCamera);
	for (int camId = 0; camId < numCamera; camId++)
	{
		centroid[camId].x = 0;
		centroid[camId].y = 0;
		centroid[camId].z = 0;

		for (int cornerId = 0; cornerId < numPoint; cornerId++)
		{
			centroid[camId].x += p[camId][cornerId].x;
			centroid[camId].y += p[camId][cornerId].y;
			centroid[camId].z += p[camId][cornerId].z;
		}

		centroid[camId].x /= numPoint;
		centroid[camId].y /= numPoint;
		centroid[camId].z /= numPoint;

		for (int cornerId = 0; cornerId < numPoint; cornerId++)
		{
			p[camId][cornerId].x -= centroid[camId].x;
			p[camId][cornerId].y -= centroid[camId].y;
			p[camId][cornerId].z -= centroid[camId].z;
		}
	}

	// compute correlation matrix H
	cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
	for (int cornerId = 0; cornerId < numPoint; cornerId++)
	{
		H.at<double>(0, 0) += p[0][cornerId].x * p[1][cornerId].x;
		H.at<double>(0, 1) += p[0][cornerId].x * p[1][cornerId].y;
		H.at<double>(0, 2) += p[0][cornerId].x * p[1][cornerId].z;

		H.at<double>(1, 1) += p[0][cornerId].y * p[1][cornerId].y;
		H.at<double>(1, 2) += p[0][cornerId].y * p[1][cornerId].z;

		H.at<double>(2, 2) += p[0][cornerId].z * p[1][cornerId].z;
	}

	H.at<double>(1, 0) = H.at<double>(0, 1);
	H.at<double>(2, 0) = H.at<double>(0, 2);
	H.at<double>(2, 1) = H.at<double>(1, 2);

	cv::SVD svd;
	cv::Mat w, ut, u, vt, v;
	svd.compute(H, w, u, vt, cv::SVD::FULL_UV);
	cv::transpose(u, ut);
	cv::transpose(vt, v);
	if (cv::determinant(u) * cv::determinant(v) < 0)
	{
		for (int i = 0; i < 3; i++)
			v.at<double>(i, 2) *= -1;
	}

	// compute rotation
	cv::Mat R(3, 3, CV_64F);
	R = v * ut;

	cv::Mat T(3, 1, CV_64F);
	cv::Mat c0(3, 1, CV_64F);
	c0.at<double>(0) = centroid[0].x;
	c0.at<double>(1) = centroid[0].y;
	c0.at<double>(2) = centroid[0].z;
	cv::Mat c1(3, 1, CV_64F);
	c1.at<double>(0) = centroid[1].x;
	c1.at<double>(1) = centroid[1].y;
	c1.at<double>(2) = centroid[1].z;
	
	// compute translation
	T = c1 - R * c0;

	M = cv::Mat::eye(4, 4, CV_32F);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			M.at<float>(i, j) = R.at<double>(i, j);
		M.at<float>(i, 2) = T.at<double>(i);
	}
}
