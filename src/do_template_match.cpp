#include "template_match.hpp"
#include "buffered_video.hpp"


int main(int argc, char** argv) {

	if (argc < 2) {
		std::cout << "Usage: template_match <video_name>" << std::endl;
		return 1;
	}

	std::cout << "Loading templates..." << std::flush;
	TemplateMatcher matcher("templates/2");
	std::cout << "done." << std::endl;

	BufferedVideo video(argv[1]);

	Frame cur_frame(video.next());

	cv::Point2i match = matcher.match(cur_frame.frame);

	cv::circle(cur_frame.frame, match, 20, cv::Scalar(0, 255, 0), 3);

	cv::imshow("Match", cur_frame.frame);
	cv::waitKey(0);

}


