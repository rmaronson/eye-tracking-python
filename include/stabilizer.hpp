/*
 * stabilizer.hpp
 *
 *  Created on: Aug 13, 2017
 *      Author: reubena
 */

#ifndef INCLUDE_STABILIZER_HPP_
#define INCLUDE_STABILIZER_HPP_

#include <vector>
#include <boost/optional.hpp>

#include "buffered_video.hpp"

struct Stabilizer {
	Stabilizer();

	cv::Mat stabilize(Frame const & frame) const;

	static boost::optional<Stabilizer> load_from_file(std::string const & infile);

private:

	std::vector<cv::Mat> transforms;


};



#endif /* INCLUDE_STABILIZER_HPP_ */
