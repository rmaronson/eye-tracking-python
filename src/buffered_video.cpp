
#include "buffered_video.hpp"

BufferedVideo::BufferedVideo(char * filename) : buf_len(0), video(filename), frame_buffer(), current_frame(), last_id(-1), filename(filename) {
	// Check video is open
	if(!video.isOpened()) {
		throw std::runtime_error((std::string("Failed to open video: ") + std::string(filename)).c_str());
	}
}

Frame const & BufferedVideo::cur() {
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

Frame const & BufferedVideo::next() {
    std::cout << "getting next frame" << std::endl;
	if (this->current_frame != IteratorType()) {
	    std::cout << "valid it, decrementing frame" << std::endl;
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
	    std::cout << "invalid it, loading frame" << std::endl;
		if (this->load_next()) {
			this->current_frame = this->frame_buffer.begin();
			return *this->current_frame;
		} else {
			throw std::out_of_range("End of video");
		}
	}
}

Frame const & BufferedVideo::prev() {
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

std::string const & BufferedVideo::get_filename() const {
	return this->filename;
}


double BufferedVideo::get_property(int val) const {
	return this->video.get(val);
}


bool BufferedVideo::load_next() {
	Frame next_frame;

	std::cout << "Reading frame..." << std::endl;
	bool read = this->video.read(next_frame.frame);
	if (read) {
		next_frame.id = ++this->last_id;
		++(this->buf_len);
		this->frame_buffer.push_front(std::move(next_frame));
		if (this->buf_len > BUF_LEN) {
			this->frame_buffer.pop_back();
		}
	} else {
		this->frame_buffer.pop_front();
	}
	return read;
}
