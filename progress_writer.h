#ifndef PROGRESS_WRITER_H
#define PROGRESS_WRITER_H

#include <boost/optional.hpp>

#include <functional>
#include <string>
#include <vector>
#include <iostream>

class threaded_progress_writer {
private:
	std::vector<int> ps_;
	std::string item_;
	mutable bool ended_;
public:
	boost::optional<std::function<void(float)>> application_progress_callback;

	threaded_progress_writer(const std::string& item, int n)
		: item_(item), ended_(false)
	{
		ps_.resize(n);
	}

	~threaded_progress_writer() {
		end();
	}

	void operator()(int n, int i);

	void end() const {
		if (!ended_) {
			std::cerr << std::endl;
			ended_ = true;
		}
	}
};

class progress_writer {
private:
	std::string item_;
	mutable bool ended_;
	bool silent_;
public:
	boost::optional<std::function<void(float)>> application_progress_callback;

	progress_writer() : silent_(true) {}

	progress_writer(const std::string& item)
		: item_(item), ended_(false), silent_(false) {}

	~progress_writer() {
		end();
	}

	void operator()(int i) const {
		if (!silent_) {
			std::cerr << "\r" << item_ << " " << i;
			std::cerr.flush();
			if (application_progress_callback) {
				(*application_progress_callback)((float)i / 100.f);
			}
		}
	}

	void end() const {
		if (!silent_ && !ended_) {
			std::cerr << std::endl;
			ended_ = true;
		}
	}

	threaded_progress_writer thread(int n) {
		return threaded_progress_writer(item_, n);
	}
};

#endif
