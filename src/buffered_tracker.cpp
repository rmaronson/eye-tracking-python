
#include "buffered_tracker.hpp"

#include <sstream>
#include <fstream>

BufferedTracker::BufferedTracker(std::string const & name) :
	data(),
	name(name) {
	// empty
}

void BufferedTracker::init_tracker(Frame const & frame, cv::Rect2d const & bbox) {
	std::cout << "Creating tracker for frame " << frame.id << ", bbox: " << bbox << std::endl;
	if (frame.id > this->data.size()) {
		throw std::out_of_range("can't create tracker for future frames");
	} else {
		this->data.resize(frame.id); // discard future data, since it's now invalid
	}

	this->data.emplace_back(bbox, cv::Tracker::create(TRACKER_TYPE));
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
	std::cout << "getting tracking data for frame " << frame.id << "(size=" << this->data.size() << ")" << std::endl;
	if (frame.id > this->data.size()) {
		throw std::out_of_range("can't track future frames");
	} else if (frame.id < this->data.size()) {
		// lazily create a tracker for the next frame, if it's required but isn't available
		std::cout << "checking to create tracker?" << std::endl;
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
	cv::Rect2d frame_bounds(0, 0, frame.frame.cols, frame.frame.rows);
	std::cout << "Picked " << bbox << std::endl;
	std::cout << "Frame bounds " << frame_bounds << std::endl;
	cv::Rect2d fixed_rect = (bbox & frame_bounds);
	std::cout << "Restricted rect: " << fixed_rect << std::endl;
	this->init_tracker(frame, fixed_rect);
}

cv::Rect2d const & BufferedTracker::get_bounding_box(int frame_id) const {
	return this->data.at(frame_id).bbox;
}

std::size_t BufferedTracker::get_last_frame_id() const {
	return this->data.size();
}

std::vector<BufferedTracker> BufferedTracker::create_trackers(Frame const & frame) {
    std::vector<cv::Rect2d> boxes;
    cv::selectROI("Choose all frames", frame.frame, boxes, false);
    cv::destroyWindow("Choose all frames");
	cv::Rect2d frame_bounds(0, 0, frame.frame.cols, frame.frame.rows);

    // Initialize tracker with first frame and bounding box
    std::vector<BufferedTracker> trackers;

    std::for_each(boxes.begin(), boxes.end(), [&trackers, &frame, &frame_bounds] (cv::Rect2d const & box) {
    	BufferedTracker tracker(std::to_string(trackers.size()));
    	tracker.init_tracker(frame, box & frame_bounds);
    	trackers.push_back(tracker);
    });
    // Add empty trackers
    for (int i=trackers.size(); i<10; ++i) {
    	BufferedTracker tracker(std::to_string(trackers.size()));
    	tracker.hold_tracker(frame);
    	trackers.push_back(tracker);
    }

    return trackers;
}


void BufferedTracker::add_bounding_box(int frame_id, cv::Rect2d && bbox) {
	if (frame_id >= this->data.size()) {
		this->data.resize(frame_id);
		this->data.emplace_back(bbox, TrackerPtr());
	} else {
		this->data[frame_id] = TrackerData(bbox, TrackerPtr());
	}
}

std::string get_csv_filename(std::string const & filename) {
    std::size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos) return filename;
    return filename.substr(0, lastdot) + std::string(".csv");
}


void BufferedTracker::save_tracking_info(std::vector<BufferedTracker> const & trackers, std::string const & filename) {
	std::string outfile(get_csv_filename(filename));
	std::ofstream outstream(outfile.c_str());

	outstream << "frame_id";
	for (int i=0; i<trackers.size(); ++i) {
		outstream << ",x" << i << ",y" << i << ",w" << i << ",h" << i;
	}
	outstream << std::endl;

	bool cont = true;
	for (int frame_id = 0; cont; ++frame_id) {
		bool stop = true;
		outstream << frame_id;
		for (int i=0; i<trackers.size(); ++i) {
			if (frame_id >= trackers[i].get_last_frame_id()) {
				outstream << ",0,0,0,0";
			} else {
				stop = false;
				cv::Rect2d const & bbox = trackers[i].get_bounding_box(frame_id);
				outstream << "," << bbox.x << "," << bbox.y << ',' << bbox.width << "," << bbox.height;
			}
		}
		outstream << '\n';
		if (stop) cont = false;
	}
	outstream << std::flush;
}

bool BufferedTracker::load_tracking_info(std::string const & filename, std::vector<BufferedTracker> & trackers) {
	std::string infile(get_csv_filename(filename));
	std::ifstream instream(infile.c_str());
	std::string line;
	std::getline(instream, line, '\n');
	if (instream.good()) {
		std::istringstream linestream(line);

		std::cout << "Processing line:\n" << line << std::endl;

		std::string token;
		std::getline(linestream, token, ',');
		if (!linestream.good() || token != "frame_id") {
			std::cout << "Bad format at first header for file " << infile << std::endl;
			return false;
		}

		std::uint8_t idx = 0;
		char const * const prefix = "xywh";
		std::size_t tracker_num = 0;
		while (linestream.good()) {
			std::getline(linestream, token, ',');
			if (token != std::string(1, prefix[idx]) + std::to_string(tracker_num)) {
				std::cout << "Unexpected header: " << token << " for file " << infile << std::endl;
				return false;
			}
			if (idx == 0) {
				trackers.emplace_back(std::to_string(tracker_num));
			}
			if (idx == 3) {
				++tracker_num;
			}
			idx = ++idx % 4;
		}
		if (idx != 0 ) {
			std::cout << "failed to process headers: bad checksum" << std::endl;
			return false;
		}
//		std::cout << "read headers for " << tracker_num << " trackers" << std::endl;

		// Read the data
		while (true) {
			std::getline(instream, line, '\n');
//			std::cout << "Processing line:\n" << line << std::endl;
			if (!instream.good()) break;

			std::istringstream linestream(line);
			std::string token;
			std::getline(linestream, token, ',');
			if (!linestream.good()) break;

			std::size_t frame_index = std::stoi(token);
			std::size_t tracker_index = 0;
			std::uint8_t prop_index = 0;
			double properties[4];

			while (linestream.good()) {
				std::getline(linestream, token, ',');
//				std::cout << "Processing token: " << token << "(" << tracker_index << ":" << static_cast<int>(prop_index) << ")" << std::endl;


				properties[prop_index] = std::stod(token);
				++prop_index;

				if (prop_index == 4) {
					prop_index = 0;
					trackers[tracker_index].add_bounding_box(frame_index, cv::Rect2d(properties[0], properties[1], properties[2], properties[3]));
					++tracker_index;
				}
			}
			if (prop_index != 0) {
				std::cout << "checksum failed, ended at strange prop num" << std::endl;
				return false;
			}
		}

		std::cout << "Read " << trackers.size() << " trackers, each with " << trackers[0].get_last_frame_id() << " frames" << std::endl;
		return true;

	} else {
		std::cout << "Failed to open file: " << infile << std::endl;
		return false;
	}

}

