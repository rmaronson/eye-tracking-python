
#include "buffered_tracker.hpp"

BufferedTracker::BufferedTracker(std::string const & name) :
	data(),
	name(name) {
	// empty
}

void BufferedTracker::init_tracker(Frame const & frame, cv::Rect2d const & bbox) {
	if (frame.id > this->data.size()) {
		throw std::out_of_range("can't create tracker for future frames");
	} else {
		this->data.resize(frame.id); // discard future data, since it's now invalid
	}

	this->data.emplace_back(TrackerData(bbox, cv::Tracker::create(TRACKER_TYPE)));
	this->data.back().tracker->init(frame.frame, bbox);
}

void BufferedTracker::hold_tracker(Frame const & frame) {
	if (frame.id > this->data.size()) {
		throw std::out_of_range("can't create tracker for future frames");
	} else {
		this->data.resize(frame.id); // discard future data, since it's now invalid
	}

	this->data.emplace_back(cv::Rect2d(), TrackerPtr());

}

cv::Rect2d const & BufferedTracker::track(Frame const & frame) {
	if (frame.id > this->data.size()) {
		throw std::out_of_range("can't track future frames");
	} else if (frame.id > -1 && frame.id < this->data.size()) {
		// lazily create a tracker for the next frame, if it's required but isn't available
		if (frame.id == this->data.size()-1 && this->data.back().bbox.area() > 0 && !this->data.back().tracker) {
			this->data.back().tracker = cv::Tracker::create(TRACKER_TYPE);
			this->data.back().tracker->init(frame.frame, this->data.back().bbox);
		}
		return this->data[frame.id].bbox;
	} else {

		TrackerData next_data;
		next_data.tracker = this->data.back().tracker;
		if (next_data.tracker) {
			next_data.tracker->update(frame.frame, next_data.bbox);
		}
		this->data.push_back(std::move(next_data));


		return this->data.back().bbox;
	}
}

void BufferedTracker::draw(Frame & frame, bool selected) {
	cv::Rect2d const & bbox = this->track(frame);
	if (bbox.width > 0 && bbox.height > 0) {
		cv::Scalar color(255, 0, 0);
		if (selected) {
			color = cv::Scalar(0, 0, 255);
		}
		cv::rectangle(frame.frame, bbox, color, 2, 1 );
		cv::putText(frame.frame, this->name, cv::Point2d(bbox.x, bbox.y+bbox.height), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, color);
	}
}

void BufferedTracker::select_new_box(Frame const & frame) {
	std::string win_name = std::string("Choose frame for ") + this->name;
	cv::Rect2d bbox = cv::selectROI(win_name, frame.frame, true, false);
	cv::destroyWindow(win_name);
	this->init_tracker(frame, bbox);
}

cv::Rect2d const & BufferedTracker::get_bounding_box(int frame_id) const {
	return this->data[frame_id].bbox;
}

std::size_t BufferedTracker::get_last_frame_id() const {
	return this->data.size();
}
