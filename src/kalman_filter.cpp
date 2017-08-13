/*
 * kalman_filter.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: reubena
 */

#include "kalman_filter.hpp"


KalmanFilter::KalmanFilter(cv::Mat const & x, cv::Mat const & P, cv::Mat const & A, cv::Mat const & H, cv::Mat const & Q) :
	x(x), P(P), A(A), H(H), Q(Q) {

}

KalmanFilter KalmanFilter::create_3d_const_accel(double const dt, cv::Mat const & init_x, cv::Mat const & Q) {
	cv::Mat x = (cv::Mat_<double>(9,1) << init_x.at<double>(0), init_x.at<double>(1), init_x.at<double>(2), 0, 0, 0, 0, 0 ,0);
	cv::Mat P(cv::Mat::eye(9, 9, CV_64F));
	double const dt2 = .5*dt*dt;
	cv::Mat A = (cv::Mat_<double>(9,9) << 1, 0, 0, dt, 0, 0, dt2, 0, 0,
			0, 1, 0, 0, dt, 0, 0, dt2, 0,
			0, 0, 1, 0, 0, dt, 0, 0, dt2,
			0, 0, 0, 1, 0, 0, dt, 0, 0,
			0, 0, 0, 0, 1, 0, 0, dt, 0,
			0, 0, 0, 0, 0, 1, 0, 0, dt,
			0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1);
	cv::Mat H(cv::Mat::eye(3, 9, CV_64F));

	return KalmanFilter(x, P, A, H, Q);
}


// http://campar.in.tum.de/Chair/KalmanFilter
cv::Mat KalmanFilter::predict() {
	x = A*x;
	P = A*P*A.t() + Q;

	return H*x;
}
cv::Mat KalmanFilter::correct(cv::Mat const & z, cv::Mat const & R) {
	cv::Mat K = P*H.t()*(H*P*H.t() + R).inv();
	x = x + K*(z - H*x);

	cv::Mat KH = K*H;
	P = (cv::Mat::eye(KH.size(), CV_64F) - KH)*P;

	return H*x;
}
