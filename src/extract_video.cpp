#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "buffered_video.hpp"
#include "buffered_tracker.hpp"


int main(int argc, char **argv)
{
	if (argc < 3) {
		std::cout << "Usage: extract_video <in_video> <out_video>" << std::endl;
		return 1;
	}
    // Read video
    BufferedVideo video(argv[1]);

    std::vector<BufferedTracker> trackers;
    if (!BufferedTracker::load_tracking_info(argv[1], trackers)) {
    	std::cout << "Failed to load tracking info!" << std::endl;
    	return 1;
    }
    std::cout << "Load complete" << std::endl;

    cv::VideoWriter outputVideo(argv[2], video.get_property(cv::CAP_PROP_FOURCC), video.get_property(cv::CAP_PROP_FPS),
    		cv::Size(video.get_property(cv::CAP_PROP_FRAME_WIDTH), video.get_property(cv::CAP_PROP_FRAME_HEIGHT)), true);
    std::cout << "Opened file for writing: " << argv[2] << std::endl;

    try {
    	while (true) {
    		Frame cur_frame(video.next());
    		for (int i=0; i<trackers.size(); ++i) {
    	    	trackers[i].draw(cur_frame, false);
    		}
    		outputVideo << cur_frame.frame;
    	}
    } catch (std::out_of_range & ex) {
    	std::cout << "complete" << std::endl;
    }

}
