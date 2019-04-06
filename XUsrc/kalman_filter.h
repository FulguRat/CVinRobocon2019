#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;

class kalman_filter
{
public:
	kalman_filter();
	kalman_filter(const kalman_filter&) = delete;
	kalman_filter& operator=(const kalman_filter&) = delete;
	~kalman_filter();

	//-- ¿¨¶ûÂüÂË²¨Ïà¹Ø
	void initKalmanFilter(int dynamParams, int measureParams, Mat transitionMatrix, Mat measurementMatrix,
		double processNoiseCov, double measurementNoiseCov, double errorCovPost);
	Mat predictAndCorrect(Mat measurement);

private:
	Mat state;

	Mat measurement;

	KalmanFilter KF;
};

