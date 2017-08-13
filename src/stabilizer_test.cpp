/*
 * stabilizer_test.cpp
 *
 *  Created on: Aug 13, 2017
 *      Author: reubena
 */


#include "stabilizer.hpp"
#include "buffered_video.hpp"


int main(int argc, char **argv) {
	// Set up tracker.
	// Instead of MIL, you can also use
	// BOOSTING, KCF, TLD, MEDIANFLOW or GOTURN

	if (argc < 2) {
		std::cout << "Usage: stabilizer_test <video>" << std::endl;
		return 1;
	}
	// Read video
	BufferedVideo video(argv[1]);
	boost::optional<Stabilizer> stabilizer = Stabilizer::load_from_file(argv[1]);

	if (stabilizer) {
		try {
			while (true) {
				Frame cur = video.next();
				cv::imshow("Video", stabilizer->stabilize(cur));
				cv::waitKey(1);
			}
		} catch (std::out_of_range & e) {

		}
	}

}



