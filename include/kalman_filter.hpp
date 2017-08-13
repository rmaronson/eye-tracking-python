/*
 * kalman_filter.hpp
 *
 *  Created on: Aug 11, 2017
 *      Author: reubena
 */

#ifndef INCLUDE_KALMAN_FILTER_HPP_
#define INCLUDE_KALMAN_FILTER_HPP_

#include <opencv2/opencv.hpp>

struct KalmanFilter {

	KalmanFilter(cv::Mat const & x, cv::Mat const & P, cv::Mat const & A, cv::Mat const & H, cv::Mat const & Q);


	cv::Mat predict();
	cv::Mat correct(cv::Mat const & z, cv::Mat const & R);

	static KalmanFilter create_3d_const_accel(double const dt, cv::Mat const & init_x, cv::Mat const & Q);



private:
	cv::Mat x;
	cv::Mat P;
	cv::Mat A;
	cv::Mat H;
	cv::Mat Q;

};



#endif /* INCLUDE_KALMAN_FILTER_HPP_ */
