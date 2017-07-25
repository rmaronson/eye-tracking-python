#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

typedef cv::Ptr<cv::Tracker> TrackerPtr;

constexpr int BUF_LEN = 30;


struct Frame {
	Frame() : frame(), id() {};
	Frame(Frame const & other) : frame(other.frame.clone()), id(other.id) {}
	cv::Mat frame;
	int id;
};

struct BufferedVideo {
	BufferedVideo(char * filename) : buf_len(0), video(filename), frame_buffer(), current_frame(), last_id(-1) {
	    // Check video is open
	    if(!video.isOpened()) {
	        throw std::runtime_error((std::string("Failed to open video: ") + std::string(filename)).c_str());
	    }
	}

	Frame const & cur() {
		if (this->current_frame != IteratorType()) {
			return *this->current_frame;
		} else {
			if (this->load_next()) {
				this->current_frame = this->frame_buffer.begin();
				return *this->current_frame;
			} else {
				throw std::out_of_range("End of video");
			}
		}
	}

	Frame const & next() {
		if (this->current_frame != this->frame_buffer.end()) {
			--(this->current_frame);
			if (this->current_frame != this->frame_buffer.end()) {
				return *this->current_frame;
			} else {
				if (this->load_next()) {
					this->current_frame = this->frame_buffer.begin();
					return *this->current_frame;
				} else {
					++(this->current_frame);
					throw std::out_of_range("End of video");
				}
			}
		} else {
			if (this->load_next()) {
				this->current_frame = this->frame_buffer.begin();
				return *this->current_frame;
			} else {
				throw std::out_of_range("End of video");
			}
		}
	}

	Frame const & prev() {
		if (this->current_frame != this->frame_buffer.end()) {
			++(this->current_frame);
			if (this->current_frame != this->frame_buffer.end()) {
				return *this->current_frame;
			} else {
				--(this->current_frame);
				throw std::out_of_range("Past buffer");
			}
		} else {
			throw std::out_of_range("Not initialized");
		}
	}

private:

	bool load_next() {
		this->frame_buffer.emplace_front();
//		std::cout << "Reading frame..." << std::endl;
		bool read = this->video.read(this->frame_buffer.front().frame);
		if (read) {
			this->frame_buffer.front().id = ++this->last_id;
			++(this->buf_len);
			if (this->buf_len > BUF_LEN) {
				this->frame_buffer.pop_back();
			}
		} else {
			this->frame_buffer.pop_front();
		}
		return read;
	}

	int buf_len;
	int last_id;
	cv::VideoCapture video;
	typedef std::list<Frame> ListType;
	ListType frame_buffer;
	typedef ListType::iterator IteratorType;
	IteratorType current_frame;
};

const char * TRACKER_TYPE = "MIL";

struct BufferedTracker {
	BufferedTracker(std::string const & name) :
		data(),
		name(name) {
		// empty
	}

	void init_tracker(Frame const & frame, cv::Rect2d const & bbox) {
		if (frame.id > this->data.size()) {
			throw std::out_of_range("can't create tracker for future frames");
		} else {
			this->data.resize(frame.id); // discard future data, since it's now invalid
		}

		this->data.emplace_back(TrackerData(bbox, cv::Tracker::create(TRACKER_TYPE)));
		this->data.back().tracker->init(frame.frame, bbox);
	}

	void hold_tracker(Frame const & frame) {
		if (frame.id > this->data.size()) {
			throw std::out_of_range("can't create tracker for future frames");
		} else {
			this->data.resize(frame.id); // discard future data, since it's now invalid
		}

		this->data.emplace_back(cv::Rect2d(), TrackerPtr());

	}

	cv::Rect2d const & track(Frame const & frame) {
		if (frame.id > this->data.size()) {
			throw std::out_of_range("can't track future frames");
		} else if (frame.id > -1 && frame.id < this->data.size()) {
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

	void draw(Frame & frame, bool selected = false) {
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

	void select_new_box(Frame const & frame) {
		std::string win_name = std::string("Choose frame for ") + this->name;
		cv::Rect2d bbox = cv::selectROI(win_name, frame.frame, true, false);
		cv::destroyWindow(win_name);
	    this->init_tracker(frame, bbox);
	}

private:
	struct TrackerData {
		TrackerData() : bbox(), tracker() {};
		TrackerData(cv::Rect2d const & bbox, TrackerPtr && tracker) :
			bbox(bbox), tracker(tracker) {};
		cv::Rect2d bbox;
		TrackerPtr tracker;
	};
	std::vector<TrackerData> data;
	std::string name;
};

std::vector<BufferedTracker> create_trackers(Frame const & frame) {
    std::vector<cv::Rect2d> boxes;
    cv::selectROI("Choose all frames", frame.frame, boxes, false);
    cv::destroyWindow("Choose all frames");

    // Initialize tracker with first frame and bounding box
    std::vector<BufferedTracker> trackers;

    std::for_each(boxes.begin(), boxes.end(), [&trackers, &frame] (cv::Rect2d const & box) {
    	BufferedTracker tracker(std::to_string(trackers.size()));
    	tracker.init_tracker(frame, box);
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

void display_frame(Frame const & frame, std::vector<BufferedTracker> & trackers, int selected = -1) {
	Frame cur_frame(frame);
	for (int i=0; i<trackers.size(); ++i) {
    	trackers[i].draw(cur_frame, i==selected);
	}

    // Display result
    cv::imshow("Video", cur_frame.frame);
}

bool pause_mode(BufferedVideo & video, std::vector<BufferedTracker> & trackers) {
	bool cont = true;
	int selected = -1;
	while (cont) {
		std::cout << "Space (resume), Esc (quit), 0-9 (select window)" << std::endl;
		if (selected > 0) {
			std::cout << "[r]eset box, [h]old region" << std::endl;
		}
		int k = cv::waitKey(0);
//		std::cout << std::to_string(k) << std::endl;
		switch (k) {
		case 27: { // esc
			return false;
		}
		case 32: { // space
			return true;
		}
		case 2: { // left
			try {
				display_frame(video.prev(), trackers, selected);
			} catch (std::out_of_range & e) {
				std::cout << "No more buffer!" << std::endl;
			}
			break;
		}
		case 3: { //right
			try {
				display_frame(video.next(), trackers, selected);
			} catch (std::out_of_range & e) {
				std::cout << "No more video!" << std::endl;
			}
			break;
		}
		case 104: { // h
			trackers[selected].hold_tracker(video.cur());
			display_frame(video.cur(), trackers, selected);
			break;
		}
		case 114: { // r
			trackers[selected].select_new_box(video.cur());
			display_frame(video.cur(), trackers, selected);
			break;
		}
		case 48:
		case 49:
		case 50:
		case 51:
		case 52:
		case 53:
		case 54:
		case 55:
		case 56:
		case 57: {
			selected = k - 48;
			display_frame(video.cur(), trackers, selected);
		}

		}
	}
	return true;
}


int main(int argc, char **argv)
{
    // Set up tracker.
    // Instead of MIL, you can also use
    // BOOSTING, KCF, TLD, MEDIANFLOW or GOTURN

	if (argc < 2) {
		std::cout << "Usage: track_image <video>" << std::endl;
		return 1;
	}
    // Read video
    BufferedVideo video(argv[1]);


    // Initialize tracker with first frame and bounding box
    auto trackers = create_trackers(video.cur());

    bool cont = true;

	while (cont) {
		try {
			display_frame(video.next(), trackers);
		} catch (std::out_of_range & e) {
			cont = pause_mode(video, trackers);
		}

		int k = cv::waitKey(1);
		switch (k) {
		case 27: { // esc
			cont = false;
			break;
		}

		case 32: { // space
			cont = pause_mode(video, trackers);
		}

		}

	}

}
