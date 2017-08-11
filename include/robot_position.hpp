/*
 * robot_position.hpp
 *
 *  Created on: Aug 11, 2017
 *      Author: reubena
 */

#ifndef INCLUDE_ROBOT_POSITION_HPP_
#define INCLUDE_ROBOT_POSITION_HPP_

#include <string>
#include <map>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/optional.hpp>


struct RobotPositionData {

	typedef double TimeType;



	struct PositionData {
		PositionData();
		cv::Mat data;

		static const std::map<std::string, std::size_t> joint_index_map;

	private:
		friend class RobotPositionData;
		PositionData(std::vector<std::string> const & joints, std::vector<std::vector<double>> const & values);

	};

	TimeType get_start_time() const;
	TimeType get_end_time() const;
	PositionData get_position(TimeType const & timestamp) const;


	static boost::optional<RobotPositionData> load_from_file(std::string const & filename);

private:

	typedef std::map<TimeType, PositionData> ContainerType;
	typedef ContainerType::iterator IteratorType;
	typedef ContainerType::const_iterator ConstIteratorType;
	ContainerType data;

	static PositionData interpolate(TimeType const & t1, PositionData const & d1, TimeType const & t2, PositionData const & d2, TimeType const & t);

};



#endif /* INCLUDE_ROBOT_POSITION_HPP_ */
