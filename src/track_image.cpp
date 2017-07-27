#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <string>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "buffered_video.hpp"
#include "buffered_tracker.hpp"



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

std::string remove_extension(std::string const & filename) {
    std::size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos) return filename;
    return filename.substr(0, lastdot);
}

void save_tracking_info(std::vector<BufferedTracker> const & trackers, std::string const & filename) {
	std::string outfile = remove_extension(filename) + std::string(".csv");
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

//bool load_tracking_info(std::string const & filename, std::vector<BufferedTracker> & trackers) {
//	std::istream instream(filename);
//	std::string line;
//	if (std::getline(instream, line, '\n')) {
//		std::stringstream()
//
//	} else {
//		return false;
//	}
//
//}

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
		std::cout << "Space (resume), Esc (quit), <- (prev frame), -> (next frame), 0-9 (select ROI)" << std::endl;
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
		case 115: { // s
			std::cout << "Saving tracking info to file..." << std::flush;
			save_tracking_info(trackers, video.get_filename());
			std::cout << "done." << std::endl;
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
