/*
 * template_match.hpp
 *
 *  Created on: Aug 9, 2017
 *      Author: reubena
 */

#ifndef INCLUDE_TEMPLATE_MATCH_HPP_
#define INCLUDE_TEMPLATE_MATCH_HPP_

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>


struct TemplateMatcher {

	TemplateMatcher(std::string const & template_dir);

	cv::Point2i match(cv::Mat const & frame);

private:

	std::vector<cv::Mat> templates;


};



#endif /* INCLUDE_TEMPLATE_MATCH_HPP_ */
