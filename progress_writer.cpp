#include "progress_writer.h"

#include <mutex>
#include <numeric>

static std::mutex progress_mtx;

void threaded_progress_writer::operator()(int n, int i)  {
	if (silent_) return;

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
	
	int completed = std::accumulate(ps_.begin(), ps_.end(), 0);
	int total = ps_.size() * 100;

	progress_mtx.unlock();

	float progress = (float)completed / total;
	if (application_progress_callback) {
		(*application_progress_callback)(progress);
	}
}