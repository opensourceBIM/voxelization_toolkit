#include "progress.h"

#include <mutex>

static std::mutex progress_mtx;

void threaded_progress_writer::operator()(int n, int i)  {
	if (ps_[n] == i) {
		return;
	}
	ps_[n] = i;
	progress_mtx.lock();
	std::cerr << "\r" << item_ << " ";
	for (auto it = ps_.begin(); it != ps_.end(); ++it) {
		std::cerr << *it << " ";
	}
	std::cerr.flush();
	progress_mtx.unlock();
}