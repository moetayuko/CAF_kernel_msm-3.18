#ifndef _WIN_STATS_
#define _WIN_STATS_

#define CONFIG_SCHED_HMP
//#define CONFIG_CFS_BANDWIDTH

#include "win_sched.h"

#define WINDOW_STATS_RECENT		0
#define WINDOW_STATS_MAX		1
#define WINDOW_STATS_MAX_RECENT_AVG	2
#define WINDOW_STATS_AVG		3
#define WINDOW_STATS_INVALID_POLICY	4

extern unsigned int sched_ravg_window;
extern unsigned int sched_disable_window_stats;
extern unsigned int pct_task_load(struct task_struct *p);
extern unsigned int max_capacity;
extern unsigned int min_capacity;
extern unsigned int max_task_load(void);
extern void sched_account_irqtime(int cpu, struct task_struct *curr,
				 u64 delta, u64 wallclock);
extern unsigned int nr_eligible_big_tasks(int cpu);

#define SCHED_HIGH_IRQ_TIMEOUT 3

static inline void
inc_cumulative_runnable_avg(struct hmp_sched_stats *stats,
				 struct task_struct *p)
{
	u32 task_load;

	task_load = p->ravg.demand;

	stats->cumulative_runnable_avg += task_load;
}

static inline void
dec_cumulative_runnable_avg(struct hmp_sched_stats *stats,
				 struct task_struct *p)
{
	u32 task_load;

	task_load = p->ravg.demand;

	stats->cumulative_runnable_avg -= task_load;

	BUG_ON((s64)stats->cumulative_runnable_avg < 0);
}

extern void update_task_ravg(struct task_struct *p, struct rq *rq,
	                     int event, u64 wallclock, u64 irqtime);
extern void sched_account_irqtime(int cpu, struct task_struct *curr,
				 u64 delta, u64 wallclock);
extern void fixup_busy_time(struct task_struct *p, int new_cpu);

extern u64 sched_ktime_clock(void);

extern int sync_cpu;
static inline void set_window_start(struct rq *rq)
{
	int cpu = cpu_of(rq);
	struct rq *sync_rq = cpu_rq(sync_cpu);

	if (rq->hmp_rq.window_start)
		return;

	if (cpu == sync_cpu) {
		rq->hmp_rq.window_start = sched_ktime_clock();
	} else {
		raw_spin_unlock(&rq->lock);
		double_rq_lock(rq, sync_rq);
		rq->hmp_rq.window_start = cpu_rq(sync_cpu)->hmp_rq.window_start;
#ifdef CONFIG_SCHED_FREQ_INPUT
		rq->curr_runnable_sum = rq->prev_runnable_sum = 0;
		rq->nt_curr_runnable_sum = rq->nt_prev_runnable_sum = 0;
#endif
		raw_spin_unlock(&sync_rq->lock);
	}

	rq->curr->ravg.mark_start = rq->hmp_rq.window_start;
}


#endif /* _WIN_STATS_ */
