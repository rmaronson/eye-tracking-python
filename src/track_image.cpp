#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "buffered_video.hpp"
#include "buffered_tracker.hpp"
#include "stabilizer.hpp"

struct ImageTracker {

	ImageTracker(std::string const & filename) : video(filename.c_str()), stabilizer(Stabilizer::load_from_file(filename)) {

	    if (!BufferedTracker::load_tracking_info(filename, trackers)) {
	    	trackers = BufferedTracker::create_trackers(video.cur());
	    }

		this->display_frame(video.next());
	}

	void run() {
	    bool cont = this->pause_mode();
		while (cont) {
			try {
				display_frame(video.next());
			} catch (std::out_of_range & e) {
				cont = pause_mode();
			}

			int k = cv::waitKey(1);
			switch (k) {
			case 27: { // esc
				std::cout << "Close without saving? [y/n] " << std::flush;
				int k = cv::waitKey(0);
				if (k == 121) {
					cont = false;
				}
				break;
			}

			case 32: { // space
				cont = this->pause_mode();
				break;
			}

			}

		}
	}

	void display_frame(Frame const & frame, int selected = -1) {
		Frame cur_frame;
		cur_frame.id = frame.id;
		if (stabilizer) {
			cur_frame.frame = this->stabilizer->stabilize(frame);
		} else {
			cur_frame.frame = frame.frame.clone();
		}

		for (int i=0; i<trackers.size(); ++i) {
	    	this->trackers[i].draw(cur_frame, i==selected);
		}

	    // Display result
	    cv::imshow("Video", cur_frame.frame);
	}

	bool pause_mode() {
		bool cont = true;
		bool changed = true;
		int selected = -1;
		while (cont) {
			std::cout << "Space (resume), Esc (quit), <- (prev frame), -> (next frame), 0-9 (select ROI)" << std::endl;
			if (selected > 0) {
				std::cout << "[r]eset box, [h]old region" << std::endl;
			}
			int k = cv::waitKey(0);
	//		std::cout << std::to_string(k) << std::endl;
			switch (k) {
			case 27: { // esc
				if (changed) {
					std::cout << "Close without saving? [y/n] " << std::flush;
					int k = cv::waitKey(0);
					if (k == 121) {
						return false;
					} else {
						break;
					}
				} else {
					return false;
				}
			}
			case 32: { // space
				return true;
			}
			case 2: { // left
				try {
					this->display_frame(video.prev(), selected);
				} catch (std::out_of_range & e) {
					std::cout << "No more buffer!" << std::endl;
				}
				break;
			}
			case 3: { //right
				try {
					this->display_frame(video.next(), selected);
					changed = true;
				} catch (std::out_of_range & e) {
					std::cout << "No more video!" << std::endl;
				}
				break;
			}
			case 104: { // h
				if (selected > -1) {
					trackers[selected].hold_tracker(video.cur());
					this->display_frame(video.cur(), selected);
					changed = true;
				}
				break;
			}
			case 114: { // r
				if (selected > -1) {
					Frame cur;
					cur.id = video.cur().id;
					if (this->stabilizer) {
						cur.frame = this->stabilizer->stabilize(video.cur());
					}
					trackers[selected].select_new_box(cur);
					this->display_frame(video.cur(), selected);
					changed = true;
				}
				break;
			}
			case 115: { // s
				std::cout << "Saving tracking info to file..." << std::flush;
				BufferedTracker::save_tracking_info(trackers, video.get_filename());
				std::cout << "done." << std::endl;
				changed = false;
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
				this->display_frame(video.cur(), selected);
			}

			}
		}
		return true;
	}

private:
	std::vector<BufferedTracker> trackers;
	BufferedVideo video;
	boost::optional<Stabilizer> stabilizer;
};




int main(int argc, char **argv) {

	if (argc < 2) {
		std::cout << "Usage: track_image <video>" << std::endl;
		return 1;
	}

	ImageTracker tracker(argv[1]);
	tracker.run();
}
