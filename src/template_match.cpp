/*
 * template_match.cpp
 *
 *  Created on: Aug 9, 2017
 *      Author: reubena
 */

#include "template_match.hpp"

#include <boost/filesystem.hpp>

TemplateMatcher::TemplateMatcher(std::string const & template_dir) {

	boost::filesystem::path dir(template_dir);

	if (!boost::filesystem::exists(dir) || !boost::filesystem::is_directory(dir)) {
		return;
	}

	boost::filesystem::directory_iterator it(template_dir);
	boost::filesystem::directory_iterator end;

	for ( ; it != end; ++it ) {
		if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ".png") {
			this->templates.push_back(cv::imread(it->path().native()));
		}
	}

	std::cout << "Loaded " << this->templates.size() << " templates." << std::endl;
}

cv::Point2i TemplateMatcher::match(cv::Mat const & frame) {
	cv::Mat sum(cv::Mat::zeros(frame.size(), CV_32F));
	cv::imshow("Frame", frame);


	std::cout << "Frame overall size: " << frame.size() << std::endl;

	int count = 0;

	std::for_each(this->templates.begin(), this->templates.end(), [&] (cv::Mat const & templ) {
		++count;
		cv::Rect2i bounds(templ.cols/2, templ.rows/2, frame.cols - templ.cols + 1, frame.rows - templ.rows + 1);
//		std::cout << "Template size: " << templ.size() << " (bounds: " << bounds << ")" << std::endl;

		cv::Mat heatmap(cv::Mat::zeros(bounds.width, bounds.height, CV_32F));
		cv::matchTemplate(frame, templ, heatmap, CV_TM_CCOEFF);
		sum(bounds) = sum(bounds) + heatmap;

		if (count % 100 == 0) {
			std::cout << count << std::endl;
			cv::Mat sum_norm;
			cv::normalize(sum, sum_norm, 1, 0, cv::NORM_INF);
			cv::imshow("Template match", sum_norm);
			cv::waitKey(1);
		}

	});

	cv::Point2i max;
	cv::minMaxLoc(sum, nullptr, nullptr, nullptr, &max);

	cv::Mat sum_norm;
	cv::normalize(sum, sum_norm, 1, 0, cv::NORM_INF);
	cv::imshow("Template match", sum_norm);
	cv::waitKey(0);

	return max;

}



