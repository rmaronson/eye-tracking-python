/*
 * position_test.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: reubena
 */

#include <iostream>

#include "robot_position.hpp"

int main(int argc, char** argv) {

	if (argc < 2) {
		std::cout << "usage: position_test <file.csv>" << std::endl;
		return 1;
	}

	boost::optional<RobotPositionData> data = RobotPositionData::load_from_file(argv[1]);

	if (data) {
		std::cout << "Start: " << data->get_start_time() << "; Duration: " << (data->get_end_time() - data->get_start_time()) << std::endl;

		std::cout << "Start data: " << data->get_position(data->get_start_time()).data << std::endl;
		std::cout << "Pre-start data: " << data->get_position(data->get_start_time()-1).data << std::endl;
		std::cout << "Post-end data: " << data->get_position(data->get_start_time()+1).data << std::endl;
		std::cout << "Mid data: " << data->get_position(data->get_start_time()+.01).data << std::endl;
	}


}
