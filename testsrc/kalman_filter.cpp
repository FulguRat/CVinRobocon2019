#include "kalman_filter.h"

kalman_filter::kalman_filter()
{
}

kalman_filter::~kalman_filter()
{
}

void kalman_filter::initKalmanFilter(int dynamParams, int measureParams, Mat transitionMatrix, Mat measurementMatrix,
	double processNoiseCov, double measurementNoiseCov, double errorCovPost)
{


	state = Mat::zeros(dynamParams, 1, CV_32F);

	KF.init(dynamParams, measureParams);

	KF.statePost = state;

	KF.transitionMatrix = transitionMatrix;

	KF.measurementMatrix = measurementMatrix;
	setIdentity(KF.processNoiseCov, Scalar::all(processNoiseCov));
	setIdentity(KF.measurementNoiseCov, Scalar::all(measurementNoiseCov));
	setIdentity(KF.errorCovPost, Scalar::all(errorCovPost));

	measurement = Mat::zeros(measureParams, 1, CV_32F);
}

Mat kalman_filter::predictAndCorrect(Mat measurement)
{
	KF.predict();
	KF.correct(measurement);

	return KF.statePost;
}