/*
 * convert_pnp.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: reubena
 */

#include <iostream>

#include "buffered_video.hpp"
#include "buffered_tracker.hpp"
#include "robot_position.hpp"
#include "kalman_filter.hpp"

int main(int argc, char** argv) {

	if (argc < 3) {
		std::cout << "Usage: convert_pnp <video> <traj_data>" << std::endl;
		return 1;
	}


    BufferedVideo video(argv[1]);

    std::vector<BufferedTracker> trackers;
    if (!BufferedTracker::load_tracking_info(argv[1], trackers)) {
    	std::cout << "Failed to load trackers" << std::endl;
    	return 1;
    }

    boost::optional<RobotPositionData> position_data = RobotPositionData::load_from_file(argv[2]);
    if (!position_data) {
    	std::cout << "Failed to read robot position data" << std::endl;
    	return 1;
    }

    double const fps = video.get_property(cv::CAP_PROP_FPS);

    std::map<std::size_t, std::string> const tracker_joint_map {
		{1, "mico_link_1"},
		{2, "mico_link_2"},
		{3, "mico_link_3"},
		{4, "mico_link_4"},
		{5, "mico_link_5"}
//		{6, "mico_link_hand"}
    };

    const double w = video.get_property(cv::CAP_PROP_FRAME_WIDTH);
    const double h = video.get_property(cv::CAP_PROP_FRAME_HEIGHT);

    cv::Mat camera_cal = (cv::Mat_<double>(3,3) << w, 0, w/2, 0, h, h/2, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_32F);
//    dist_coeffs.at<double>(0,0) = 0;
//    dist_coeffs.at<double>(1,0) = 0;
//    dist_coeffs.at<double>(2,0) = 0;
//    dist_coeffs.at<double>(3,0) = 0;

    std::cout << "cal:\n" << camera_cal << std::endl;

	cv::Mat rvec = (cv::Mat_<double>(3,1) << 0.74, 2.82, -1.11);
	cv::Mat tvec = (cv::Mat_<double>(3,1) << 0.18, 0.29, 0.65);

	cv::Mat Q = cv::Mat::eye(9, 9, CV_64F);
	Q(cv::Range(3,6), cv::Range(3,6)) *= .02*fps;
	Q(cv::Range(6,9), cv::Range(6,9)) *= .004*fps*fps;

//    KalmanFilter kf_t = KalmanFilter::create_3d_const_accel(1./fps, rvec, Q);
    KalmanFilter kf_t = KalmanFilter::create_3d_const_accel(1./fps, tvec, Q);

    try {
    	while (true) {
    		Frame cur(video.next());

    		RobotPositionData::TimeType timestamp = position_data->get_start_time() + cur.id*fps;

    		std::vector<std::size_t> tracker_indices;
    		std::vector<cv::Rect2d> bboxes;

    		std::for_each(tracker_joint_map.begin(), tracker_joint_map.end(), [&] (std::map<std::size_t, std::string>::value_type const & pair) {
    			cv::Rect2d bbox(trackers[pair.first].get_bounding_box(cur.id));
    			if (bbox.area() > 0){
    				tracker_indices.push_back(pair.first);
    				trackers[pair.first].draw(cur);
    				bboxes.push_back(std::move(bbox));
    			}
    		});

    		tvec = kf_t.predict();
			RobotPositionData::PositionData full_pos(position_data->get_position(timestamp));

			std::vector<cv::Point2f> reprojected;

    		if (!tracker_indices.empty()) {

				std::vector<cv::Point2f> points2d;
				std::vector<cv::Point3f> points3d;
				std::size_t j(0);
				std::for_each(tracker_indices.begin(), tracker_indices.end(), [&] (std::size_t idx) {
	//    				std::cout << "Getting data for idx " << idx << ":" << std::flush;
	//    				std::cout << "\tjoint: " << tracker_joint_map.at(idx) << ", idx: " << RobotPositionData::PositionData::joint_index_map.at(
	//							tracker_joint_map.at(idx)
	//						) << std::endl;
					points3d.push_back(cv::Point3f(full_pos.data.col(
							RobotPositionData::PositionData::joint_index_map.at(
										tracker_joint_map.at(idx)
									)
							)));
	//    				std::cout << points3d.back() << std::endl;
					points2d.push_back(bboxes[j].tl() + cv::Point2d(bboxes[j].width/2, bboxes[j].height/2));
	//    				std::cout << points2d.back() << std::endl;
					++j;
				});


				if (tracker_indices.size() >= 3) {

					std::cout << "\n\n" << "3d:\n" << points3d << "\n2d:\n" << cv::Mat(points2d) << std::endl;
					bool solved = cv::solvePnP(points3d, cv::Mat(points2d), camera_cal, dist_coeffs, rvec, tvec, true);

					if (solved) {
						cv::projectPoints(points3d, rvec, tvec, camera_cal, dist_coeffs, reprojected);


						std::for_each(reprojected.begin(), reprojected.end(), [&] (cv::Point2f const & pt) {
							if (pt.x >= 0 && pt.x <= w && pt.y >= 0 && pt.y <= h) {
								cv::circle(cur.frame, pt, 5, cv::Scalar(0, 255, 255), 2);
							}
						});

						std::cout << "Computed: r:\n" << rvec << "\nt:\n" << tvec << "\nReprojected:\n" << cv::Mat(reprojected) << std::endl;
						cv::Mat diffs = (cv::Mat(reprojected) - cv::Mat(points2d));
						double const diff = cv::norm(diffs);
						std::cout << "diff mat:\n" << diffs << "\ndiff: " << diff << std::endl;
						tvec = kf_t.correct(tvec, diff*cv::Mat::eye(3, 3, CV_64F)/(.5*(w+h)*tracker_indices.size()));
						reprojected.clear();

					}
				} else {
					std::cout << "Not enough points for transform" << std::endl;
				}
				std::for_each(points2d.begin(), points2d.end(), [&] (cv::Point2f const & pt) {
					if (pt.x >= 0 && pt.x <= w && pt.y >= 0 && pt.y <= h) {
						cv::circle(cur.frame, pt, 3, cv::Scalar(0, 0, 255), 1);
					}
				});
    		}

			std::cout << "Resolved:\nr:\n" << rvec << "\nt:\n" << tvec << std::endl;
			// Project the points back into the frame

			std::vector<cv::Point3f> points3d_all;
			for (int i=0; i<full_pos.data.cols; ++i) {
				points3d_all.push_back(cv::Point3f(full_pos.data.col(i)));
			}

			cv::projectPoints(points3d_all, rvec, tvec, camera_cal, dist_coeffs, reprojected);

			std::for_each(reprojected.begin(), reprojected.end(), [&] (cv::Point2f const & pt) {
				if (pt.x >= 0 && pt.x <= w && pt.y >= 0 && pt.y <= h) {
					cv::circle(cur.frame, pt, 5, cv::Scalar(0, 255, 0), 2);
				}
			});

    		cv::imshow("Frame", cur.frame);
    		cv::waitKey(0);


    	}
    } catch (std::out_of_range & e) {
    	return 0;
    }
}



