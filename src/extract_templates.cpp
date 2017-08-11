#include <iostream>
#include <iomanip>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include "buffered_video.hpp"
#include "buffered_tracker.hpp"

std::string get_output_filename(std::string const & filename, boost::filesystem::path const & dir, std::size_t const frame_id) {
    boost::filesystem::path video_path(filename);
    std::ostringstream ss;
    ss << video_path.stem().native() << "_" << std::setfill('0') << std::setw(6) << frame_id << ".png";
    return (dir / ss.str()).native();
}

void extract_from_video(std::string const & name, boost::filesystem::path const & output_dir) {
    BufferedVideo video(name.c_str());

    std::vector<BufferedTracker> trackers;
    if (!BufferedTracker::load_tracking_info(name.c_str(), trackers)) {
    	std::cout << "Failed to load tracking info for " << name << std::endl;
    	return;
    }
    std::cout << "Loaded " << name << std::endl;

    try {
    	while (true) {
    		Frame cur_frame(video.next());
    		for (int i=0; i<trackers.size(); ++i) {
    			boost::filesystem::path cur_path = output_dir / boost::lexical_cast<std::string>(i);
    			if (!boost::filesystem::exists(cur_path)) {
    				boost::filesystem::create_directory(cur_path);
    			}
    			cv::Rect2d bbox(trackers[i].get_bounding_box(cur_frame.id));
    			if (bbox.area() > 0) {
    				cv::imwrite(get_output_filename(name, cur_path, cur_frame.id), cur_frame.frame(bbox));
    			}
    		}
    	}
    } catch (std::out_of_range & ex) {
    	std::cout << "complete" << std::endl;
    }

}

int main(int argc, char **argv)
{
	if (argc < 3) {
		std::cout << "Usage: extract_templates <out_dest> <in_videos>" << std::endl;
		return 1;
	}

	boost::filesystem::path out_dest(argv[1]);

	for (int i=2; i<argc; ++i) {
		extract_from_video(argv[i], out_dest);
	}

}
