/*
 * stabilizer.cpp
 *
 *  Created on: Aug 13, 2017
 *      Author: reubena
 */

#include "stabilizer.hpp"

Stabilizer::Stabilizer() : transforms() {}


cv::Mat Stabilizer::stabilize(Frame const & frame) const {

	static std::size_t const offset_w = 500;
	static std::size_t const offset_h = 500;

//	std::cout << "Getting tf for " << frame.id << std::endl;

	cv::Mat transform = this->transforms.at(frame.id).clone();

//	std::cout << "Init:\n" << transform << std::endl;

	cv::Mat center = (cv::Mat_<double>(3,3) << 1, 0, frame.frame.cols/2 + offset_w/2,
											   0, 1, frame.frame.rows/2 + offset_w/2,
											   0, 0, 1);
	cv::Mat inv_center = (cv::Mat_<double>(3,3) << 1, 0, -frame.frame.cols/2,
									  		       0, 1, -frame.frame.rows/2,
											       0, 0, 1);

	transform = center*transform*inv_center;

//	std::cout << "Fixed:\n" << transform << std::endl;

	cv::Mat output;
	cv::warpAffine(frame.frame, output, transform.rowRange(0, 2), cv::Size(frame.frame.cols+offset_w, frame.frame.rows+offset_h));

	return output;
}

std::string get_csv_filename(std::string const & filename) {
    std::size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos) return filename;
    return filename.substr(0, lastdot) + std::string("_tf.csv");
}


boost::optional<Stabilizer> Stabilizer::load_from_file(std::string const & infile) {
	std::ifstream instream(get_csv_filename(infile));
	std::cout << "Loading file " << get_csv_filename(infile) << std::endl;
	std::string line;
	std::getline(instream, line, '\n');

	if (instream.good()) {
		std::istringstream linestream(line);

		if (line != "frame,tx,ty,theta,s") {
			std::cout << "Bad format at first header for file " << infile << std::endl;
			return boost::optional<Stabilizer>();
		}

		// Read the data
		Stabilizer stabilizer;
		cv::Mat const id = cv::Mat::eye(3,3, CV_64FC1);

		cv::Mat last_mat = id;

		std::size_t const max_skip(5);
		char delim;

		while (true) {
			std::size_t cur_frame;
			double tx, ty, th, s;

			std::getline(instream, line, '\n');
			std::cout << "Processing line:\n" << line << std::endl;
			if (!instream.good() || line.empty()) break;

			std::istringstream linestream(line);

			linestream >> cur_frame;
			linestream >> delim;
			if (delim != ',') {
				std::cout << "Unexpected delimiter: "  << delim << " (line: " << line << ")" << std::endl;
				return boost::optional<Stabilizer>();
			}

			linestream >> tx;
			linestream >> delim;
			if (delim != ',') {
				std::cout << "Unexpected delimiter: "  << delim << " (line: " << line << ")" << std::endl;
				return boost::optional<Stabilizer>();
			}

			linestream >> ty;
			linestream >> delim;
			if (delim != ',') {
				std::cout << "Unexpected delimiter: "  << delim << " (line: " << line << ")" << std::endl;
				return boost::optional<Stabilizer>();
			}

			linestream >> th;
			linestream >> delim;
			if (delim != ',') {
				std::cout << "Unexpected delimiter: "  << delim << " (line: " << line << ")" << std::endl;
				return boost::optional<Stabilizer>();
			}

			linestream >> s;

			// Are we resetting?
			bool reset = (cur_frame - stabilizer.transforms.size()) > max_skip;
			// Backfill
			while (stabilizer.transforms.size() < cur_frame) {
				stabilizer.transforms.push_back(last_mat.clone());
			}

			if (reset) {
				last_mat = id.clone();
			}

			double const a = s*std::cos(th);
			double const b = s*std::sin(th);
			cv::Mat cur_mat = (cv::Mat_<double>(3,3) << a, b, tx,
					-b, a, ty,
					0, 0, 1);
			std::cout << "Cur mat:\n" << cur_mat << std::endl;
			last_mat = cur_mat*last_mat;
			std::cout << "Combined mat:\n" << last_mat << std::endl;
			stabilizer.transforms.push_back(last_mat.clone());

		}

		//		std::cout << data.data.begin()->second.data << std::endl;

		return boost::optional<Stabilizer>(std::move(stabilizer));

	} else {
		return boost::optional<Stabilizer>();
	}

}



