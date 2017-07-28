#ifndef BUFFERED_TRACKER_HPP_
#define BUFFERED_TRACKER_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "buffered_video.hpp"

typedef cv::Ptr<cv::Tracker> TrackerPtr;

struct TrackerData {
	TrackerData() : bbox(), tracker() {};
	TrackerData(cv::Rect2d const & bbox, TrackerPtr && tracker) :
		bbox(bbox), tracker(tracker) {};
	cv::Rect2d bbox;
	TrackerPtr tracker;
};


struct BufferedTracker {
	BufferedTracker(std::string const & name);

	void init_tracker(Frame const & frame, cv::Rect2d const & bbox);

	void hold_tracker(Frame const & frame);

	cv::Rect2d const & track(Frame const & frame);

	void draw(Frame & frame, bool selected = false);

	void select_new_box(Frame const & frame);

	cv::Rect2d const & get_bounding_box(int frame_id) const;

	std::size_t get_last_frame_id() const;


	static constexpr char const * TRACKER_TYPE = "MIL";

	static std::vector<BufferedTracker> create_trackers(Frame const & frame);

	static void save_tracking_info(std::vector<BufferedTracker> const & trackers, std::string const & filename);
	static bool load_tracking_info(std::string const & filename, std::vector<BufferedTracker> & trackers);

private:

	void add_bounding_box(int frame_id, cv::Rect2d && bbox);

	std::vector<TrackerData> data;
	std::string name;
};


#endif
