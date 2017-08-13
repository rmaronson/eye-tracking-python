
#include <iostream>

#include "kalman_filter.hpp"


int main(int argc, char** argv) {

	cv::Mat x = (cv::Mat_<double>(3,1) << 1, 2, 3);
	cv::Mat Q = cv::Mat::eye(9, 9, CV_64F);
	Q(cv::Range(3,6), cv::Range(3,6)) *= 10;
	Q(cv::Range(6,9), cv::Range(6,9)) *= 100;

	cv::Mat R = cv::Mat::eye(3, 3, CV_64F) * 0.001;
	KalmanFilter kf = KalmanFilter::create_3d_const_accel(0.1, x, Q);

	std::cout << "0: " << x << std::endl;
	std::cout << "0.1: " << kf.predict() << std::endl;

	x.at<double>(0) = 2;
	x.at<double>(1) = 2;
	x.at<double>(2) = 2;

	kf.correct(x, R);

	std::cout << "0.2: " << kf.predict() << std::endl;


	x.at<double>(0) = 3;
	x.at<double>(1) = 2;
	x.at<double>(2) = 1;

	kf.correct(x, R);

	std::cout << "0.3: " << kf.predict() << std::endl;


	x.at<double>(0) = 4;
	x.at<double>(1) = 2;
	x.at<double>(2) = 0;

	kf.correct(x, R);

	std::cout << "0.4: " << kf.predict() << std::endl;


	x.at<double>(0) = 5;
	x.at<double>(1) = 2;
	x.at<double>(2) = -1;

	kf.correct(x, R);

	std::cout << "0.5: " << kf.predict() << std::endl;
	std::cout << "0.6: " << kf.predict() << std::endl;
	std::cout << "0.7: " << kf.predict() << std::endl;
	std::cout << "0.8: " << kf.predict() << std::endl;
	std::cout << "0.9: " << kf.predict() << std::endl;
	std::cout << "1.0: " << kf.predict() << std::endl;


}
