#ifndef PROGRESS_H
#define PROGRESS_H

#include <string>
#include <vector>
#include <iostream>

class threaded_progress_writer {
private:
	std::vector<int> ps_;
	std::string item_;
	mutable bool ended_;
public:
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
public:
	progress_writer(const std::string& item)
		: item_(item), ended_(false) {}
	~progress_writer() {
		end();
	}
	void operator()(int i) const {
		std::cerr << "\r" << item_ << " " << i;
		std::cerr.flush();
	}
	void end() const {
		if (!ended_) {
			std::cerr << std::endl;
			ended_ = true;
		}
	}
	threaded_progress_writer thread(int n) {
		return threaded_progress_writer(item_, n);
	}
};

#endif
