#include "RGBDCamera.h"

RGBDCamera::RGBDCamera() : m_intrinsic(NULL)
{

}

RGBDCamera::~RGBDCamera()
{
	clear();
}

void RGBDCamera::clear()
{
	if (!m_intrinsic)
	{
		delete m_intrinsic;
		m_intrinsic = NULL;
	}

	for (int frameId = 0; frameId < m_color.size(); frameId++)
	{
		m_color[frameId]->release();
	}
	m_color.clear();

	for (int frameId = 0; frameId < m_depth.size(); frameId++)
	{
		m_depth[frameId]->release();
	}
	m_depth.clear();

	for (int frameId = 0; frameId < m_corners2d.size(); frameId++)
	{
		m_corners2d[frameId].clear();
	}
	m_corners2d.clear();

	for (int frameId = 0; frameId < m_corners3d.size(); frameId++)
	{
		m_corners3d[frameId].clear();
	}
	m_corners3d.clear();

	m_numFrame = 0;
}

void RGBDCamera::init(const std::vector<std::string> colorFilenames,
	const std::vector<std::string> depthFilenames,
	const int& patternWidth,
	const int& patternHeight,
	const float& patternLength,
	const CameraIntrinsicF* intrinsic)
{
	clear();

	m_numFrame = colorFilenames.size();

	_loadColor(colorFilenames);
	_loadDepth(depthFilenames);
	_extractCorners2dCheckerboard(cv::Size(patternWidth, patternHeight));

	m_intrinsic = new CameraIntrinsicF;
	if (intrinsic == NULL)
	{
		// perform intrinsic calibration when needed
		printf("Need to compute intrinsic!\n");
		_computeIntrinsic(cv::Size(patternWidth, patternHeight), patternLength);
	}
	else {
		m_intrinsic->copyFrom(intrinsic);
	}
}

void RGBDCamera::_loadColor(const std::vector<std::string> colorFilenames)
{
	int numColor = colorFilenames.size();

	m_color.resize(numColor);
	for (int frameId = 0; frameId < numColor; frameId++)
	{
		cv::Mat colorMat = cv::imread(colorFilenames[frameId]);
		if (colorMat.data != NULL)
		{
			m_color[frameId] = new cv::Mat;
			*m_color[frameId] = colorMat;
		}
		else {
			m_color[frameId] = NULL;
			printf("Loading color frame %d failed! - %s\n", frameId, colorFilenames[frameId].c_str());
		}
		colorMat.release();
	}
}

void RGBDCamera::_loadDepth(const std::vector<std::string> depthFilenames)
{
	int numDepth = depthFilenames.size();

	m_depth.resize(numDepth);
	for (int frameId = 0; frameId < numDepth; frameId++)
	{
		cv::Mat depthMat = cv::imread(depthFilenames[frameId], CV_LOAD_IMAGE_ANYDEPTH); // 16-bit unsigned short
		if (depthMat.data != NULL)
		{
			m_depth[frameId] = new cv::Mat;
			*m_depth[frameId] = depthMat;
		}
		else {
			m_depth[frameId] = NULL;
			printf("Loading depth frame %d failed! - %s\n", frameId, depthFilenames[frameId].c_str());
		}
		depthMat.release();
	}
}

void RGBDCamera::_extractCorners2dCheckerboard(const cv::Size patternSize)
{
	cv::Mat grayMat;

	m_corners2d.clear();
	m_bPatternDetected.resize(m_numFrame);
	for (int frameId = 0; frameId < m_numFrame; frameId++)
	{
		// init flags
		m_bPatternDetected[frameId] = true;

		cv::cvtColor(*m_color[frameId], grayMat, CV_RGB2GRAY);

		corner2d_t frameCorners;
		frameCorners.clear();

		bool patternfound =
			cv::findChessboardCorners(grayMat,
			patternSize,
			frameCorners,
			cv::CALIB_CB_ADAPTIVE_THRESH +
			cv::CALIB_CB_NORMALIZE_IMAGE);

		if (!patternfound)
		{
			printf("Pattern not found in frame %d!\n", frameId);
			m_bPatternDetected[frameId] = false;
		}
		else {
			cv::cornerSubPix(grayMat, frameCorners, patternSize,
				cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 300, 0.01));
		}
		m_corners2d.push_back(frameCorners);

#if DEBUG_SHOW_DETECTED_CORNERS
		cv::Mat cornerShowMat = m_color[frameId]->clone();
		cv::drawChessboardCorners(cornerShowMat,
			patternSize,
			cv::Mat(frameCorners),
			1);
		cv::namedWindow("ShowCorner", 1);
		cv::imshow("ShowCorner", cornerShowMat);
		cv::waitKey(0);
#endif
	}
}

void RGBDCamera::_extractCorners3d()
{
	int w = m_intrinsic->w;
	int h = m_intrinsic->h;

	m_corners3d.resize(m_corners2d.size());
	for (int frameId = 0; frameId < m_numFrame; frameId++)
	{
		m_corners3d[frameId].resize(m_corners2d[frameId].size());

		// skip if no pattern detected
		if (!m_bPatternDetected[frameId]) continue;

		for (int cornerId = 0; cornerId < m_corners2d[frameId].size(); cornerId++)
		{
			// get 2d coord.
			cv::Point2f uf(m_corners2d[frameId][cornerId].x,
				m_corners2d[frameId][cornerId].y);

			// get depth
			cv::Point2i ui, nnui;
			ui.x = static_cast<int>(std::floor(uf.x));
			ui.y = static_cast<int>(std::floor(uf.y));

			float z = m_depth[frameId]->at<ushort>(ui.y, ui.x);
			float z_avg = 0;
			int count = 0;

			for (int det_x = 0; det_x <= DEPTH_SAMPLE_RANGE; det_x++)
				for (int det_y = 0; det_y <= DEPTH_SAMPLE_RANGE; det_y++)
				{
					nnui.x = ui.x + det_x;
					nnui.y = ui.y + det_y;

					// check if inside image
					if (nnui.x >= 0 && nnui.x < w && nnui.y >= 0 && nnui.y < h
						&& m_depth[frameId]->at<ushort>(nnui.y, nnui.x) > 0
						&& std::abs(z - m_depth[frameId]->at<ushort>(nnui.y, nnui.x)) < DEPTH_SIMILARITY_THRESHOLD)
					{
						z_avg += std::abs(z - m_depth[frameId]->at<ushort>(nnui.y, nnui.x));
						count++;
					}
				}

			// find averaged depth in mm
			z_avg /= count;

			// mm to meters
			z_avg /= 1000.0f;

			// compute 3d coord.
			m_corners3d[frameId][cornerId].z = z_avg; 
			m_corners3d[frameId][cornerId].x = (uf.x - m_intrinsic->cx) / m_intrinsic->fx * z_avg;
			m_corners3d[frameId][cornerId].y = (uf.y - m_intrinsic->cy) / m_intrinsic->fy * z_avg;	
		}
	}
}

void RGBDCamera::_computeIntrinsic(const cv::Size patternSize, const float& patternLength)
{
	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<float> reprojErrs;
	float totalAvgErr = 0;

	// get image size
	int imageWidth = m_color[0]->cols;
	int imageHeight = m_color[0]->rows;

	// get checkerboard corners
	corner3d_t corner3dRef;
	for (int j = 0; j < patternSize.height; j++)
		for (int i = 0; i < patternSize.width; i++)
			corner3dRef.push_back(cv::Point3d(float(j*patternLength), float(i*patternLength), 0));

	// get valid intrinsic points
	std::vector<corner2d_t> corners2dForIntrinsic;
	corners2dForIntrinsic.clear();
	for (int frameId = 0; frameId < m_numFrame; frameId++)
	{
		if (m_bPatternDetected[frameId])
		{
			corners2dForIntrinsic.push_back(m_corners2d[frameId]);
		}
	}

	// compute intrinsic params.
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat disCoeffs = cv::Mat::zeros(8, 1, CV_64F);
	std::vector<corner3d_t> objectPoints;
	objectPoints.resize(corners2dForIntrinsic.size(), corner3dRef);
	bool ok = cv::calibrateCamera(objectPoints,
		corners2dForIntrinsic,
		cv::Size(imageWidth, imageHeight),
		cameraMatrix,
		disCoeffs,
		rvecs, tvecs);

	// evaluate calibration results
	corner2d_t corner2dPerFrame;
	int totalPoints = 0;
	float totalError = 0, err;
	for (int frameId = 0; frameId < corners2dForIntrinsic.size(); frameId++)
	{
		cv::projectPoints(objectPoints[frameId],
			rvecs[frameId],
			tvecs[frameId],
			cameraMatrix,
			disCoeffs,
			corner2dPerFrame);
		
		err = cv::norm(corners2dForIntrinsic[frameId], corner2dPerFrame, CV_L2);

		int n = (int)objectPoints[frameId].size();
		totalError += err * err;
		totalPoints += n;
	}
	totalAvgErr = std::sqrt(totalError / totalPoints);

	printf("Reprojection Error is %f\n", totalAvgErr);

	// update result
	m_intrinsic->w = imageWidth;
	m_intrinsic->h = imageHeight;
	m_intrinsic->fx = cameraMatrix.at<double>(0, 0);
	m_intrinsic->fy = cameraMatrix.at<double>(1, 1);
	m_intrinsic->cx = cameraMatrix.at<double>(0, 2);
	m_intrinsic->cy = cameraMatrix.at<double>(1, 2);
	for (int i = 0; i < 5; i++)
		m_intrinsic->dist[i] = disCoeffs.at<double>(i);

	m_intrinsic->printParam();
}