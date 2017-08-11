#include "robot_position.hpp"

#include <fstream>
#include <iostream>

std::map<std::string, std::size_t> const RobotPositionData::PositionData::joint_index_map {
		{"mico_link_1", 0},
		{"mico_link_2", 1},
		{"mico_link_3", 2},
		{"mico_link_4", 3},
		{"mico_link_5", 4},
		{"mico_link_hand", 5}
};



boost::optional<RobotPositionData> RobotPositionData::load_from_file(std::string const & infile) {
	std::ifstream instream(infile);
	std::string line;
	std::getline(instream, line, '\n');

	if (instream.good()) {
		std::istringstream linestream(line);

//		std::cout << "Processing line:\n" << line << std::endl;

		std::string token;
		std::getline(linestream, token, ',');
		if (!linestream.good() || token != "time") {
			std::cout << "Bad format at first header for file " << infile << std::endl;
			return boost::optional<RobotPositionData>();
		}

		std::uint8_t idx = 0;
		std::vector<std::string> const suffixes {"_x", "_y", "_z"};
		std::vector<std::string> joints;

		std::string last_joint;
		while (linestream.good()) {
			std::getline(linestream, token, ',');

			if (idx == 0) {
				std::size_t loc = token.rfind(suffixes[idx]);
				if (loc == std::string::npos || loc != token.size()-suffixes[idx].size()) {
					std::cout << "Unexpected header: " << token << " for file " << infile << "(matched at: " << loc << ")" <<std::endl;
					std::cout << "Token size: " << token.size() << "; suffix size: " << suffixes[idx].size() << std::endl;
					return boost::optional<RobotPositionData>();
				} else {
					joints.push_back(token.substr(0, loc));
				}
			} else if (token != joints.back() + suffixes[idx]) {
				std::cout << "Unexpected header: " << token << " for file " << infile << std::endl;
				return boost::optional<RobotPositionData>();
			}
			idx = ++idx % suffixes.size();
		}
		if (idx != 0 ) {
			std::cout << "failed to process headers: bad checksum" << std::endl;
			return boost::optional<RobotPositionData>();
		}
//		std::cout << "read headers for " << joints.size() << " joints" << std::endl;

		// Read the data
		RobotPositionData data;
		while (true) {
			std::getline(instream, line, '\n');
//			std::cout << "Processing line:\n" << line << std::endl;
			if (!instream.good()) break;

			std::istringstream linestream(line);
			std::string token;
			std::getline(linestream, token, ',');
			if (!linestream.good()) break;

			TimeType time = std::stod(token);
			std::uint8_t prop_index = 0;
			std::vector<std::vector<double> > values;

			while (linestream.good()) {
				std::getline(linestream, token, ',');

				if (prop_index == 0) {
					values.emplace_back();
				}
//				std::cout << "Processing token: " << token << "(" << values.size() << ":" << static_cast<int>(prop_index) << "): " << std::stod(token) << std::endl;

				values.back().push_back(std::stod(token));
				prop_index = ++prop_index % suffixes.size();

			}
			if (prop_index != 0) {
				std::cout << "checksum failed, ended at strange prop num" << std::endl;
				return boost::optional<RobotPositionData>();
			}

			data.data.emplace(time, PositionData(joints, values));
		}

//		std::cout << data.data.begin()->second.data << std::endl;

		return boost::optional<RobotPositionData>(data);

	} else {
		return boost::optional<RobotPositionData>();
	}

}

RobotPositionData::PositionData::PositionData() :
	data(3, RobotPositionData::PositionData::joint_index_map.size(), CV_32F) {}

RobotPositionData::PositionData::PositionData(std::vector<std::string> const & joints, std::vector<std::vector<double> > const & values) :
	data(3, RobotPositionData::PositionData::joint_index_map.size(), CV_32F) {
	auto joint = joints.begin();
	auto value = values.begin();
	while (joint != joints.end() && value != values.end()) {
		cv::Mat(*value).copyTo(this->data.col(RobotPositionData::PositionData::joint_index_map.at(*joint)));
//		std::cout << "expected: " << cv::Mat(*value) << std::endl;
//		std::cout << "actual: " << this->data.col(RobotPositionData::PositionData::joint_index_map.at(*joint)) << std::endl;
		++joint; ++value;
	}
}


RobotPositionData::TimeType RobotPositionData::get_start_time() const {
	return this->data.begin()->first;
}

RobotPositionData::TimeType RobotPositionData::get_end_time() const {
	return this->data.rbegin()->first;
}

RobotPositionData::PositionData RobotPositionData::get_position(RobotPositionData::TimeType const & timestamp) const {
	RobotPositionData::ConstIteratorType lower = this->data.lower_bound(timestamp);

//	std::for_each(this->data.begin(), this->data.end(), [&] (RobotPositionData::ContainerType::value_type const & pair) {
//		std::cout << "t: " << std::setprecision(16) << pair.first << std::endl;
//	});

//	std::cout << "searched for " << std::setprecision(16) << timestamp << " (min: " << this->data.begin()->first << " max: " << this->data.rbegin()->first << ")" << std::endl;

	if (lower == this->data.end()) {
//		std::cout << "No lower bound found, returning end" << std::endl;
		return this->data.rbegin()->second;
	} else if (lower == this->data.begin()) {
//		std::cout << "No upper bound found, returning lower" << std::endl;
		return lower->second;
	} else if (lower->first == timestamp) {
//		std::cout << "Found exactly" << std::endl;
		return lower->second;
	} else {
//		std::cout << "Interpolating" << std::endl;
		auto pre = lower; --pre;
		return RobotPositionData::interpolate(lower->first, lower->second, pre->first, pre->second, timestamp);
	}
}

RobotPositionData::PositionData RobotPositionData::interpolate(RobotPositionData::TimeType const & t1, RobotPositionData::PositionData const & d1,
		RobotPositionData::TimeType const & t2, RobotPositionData::PositionData const & d2, RobotPositionData::TimeType const & t) {
	double const factor = (t-t1)/(t2-t1);
//	std::cout << "Interpolating btw " << std::setprecision(16) << t1 << ", " << t2 << ": d=" << (t2-t1) << " n=" << (t-t1) << " f=" << factor << std::endl;

	RobotPositionData::PositionData result;
	result.data = (1-factor)*d1.data + factor*d2.data;
	return result;
}

