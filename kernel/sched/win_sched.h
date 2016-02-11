#ifndef _WIN_SCHED_
#define _WIN_SCHED_

struct hmp_sched_stats {
	//int nr_big_tasks;
	u64 cumulative_runnable_avg;
};

struct hmp_rq {
	u64 window_start;
};

#endif /* _WIN_SCHED_ */
