#undef TRACE_SYSTEM
#define TRACE_SYSTEM alarmtimer

#if !defined(_TRACE_ALARMTIMER_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_ALARMTIMER_H

#include <linux/alarmtimer.h>
#include <linux/rtc.h>
#include <linux/tracepoint.h>

TRACE_DEFINE_ENUM(ALARM_REALTIME);
TRACE_DEFINE_ENUM(ALARM_BOOTTIME);

#define show_alarm_type(type)	__print_flags(type, " | ",	\
	{ 1 << ALARM_REALTIME, "ALARM_REALTIME" },		\
	{ 1 << ALARM_BOOTTIME, "ALARM_BOOTTIME" })

DECLARE_EVENT_CLASS(alarm_setting,

	TP_PROTO(struct rtc_time *rtc_time, int flag),

	TP_ARGS(rtc_time, flag),

	TP_STRUCT__entry(
		__field(unsigned char, second)
		__field(unsigned char, minute)
		__field(unsigned char, hour)
		__field(unsigned char, day)
		__field(unsigned char, mon)
		__field(unsigned short, year)
		__field(unsigned char, alarm_type)
	),

	TP_fast_assign(
		__entry->second = rtc_time->tm_sec;
		__entry->minute = rtc_time->tm_min;
		__entry->hour = rtc_time->tm_hour;
		__entry->day = rtc_time->tm_mday;
		__entry->mon = rtc_time->tm_mon;
		__entry->year = rtc_time->tm_year;
		__entry->alarm_type = flag;
	),

	TP_printk("alarmtimer type:%s expires time: %hu-%u-%u %u:%u:%u",
		  show_alarm_type((1 << __entry->alarm_type)),
		  __entry->year + 1900,
		  __entry->mon + 1,
		  __entry->day,
		  __entry->hour,
		  __entry->minute,
		  __entry->second
	)
);

DEFINE_EVENT(alarm_setting, alarmtimer_suspend,

	TP_PROTO(struct rtc_time *time, int flag),

	TP_ARGS(time, flag)
);

DECLARE_EVENT_CLASS(alarm_processing,

	TP_PROTO(struct alarm *alarm, char *process_name),

	TP_ARGS(alarm, process_name),

	TP_STRUCT__entry(
		__field(unsigned long long, expires)
		__field(unsigned char, second)
		__field(unsigned char, minute)
		__field(unsigned char, hour)
		__field(unsigned char, day)
		__field(unsigned char, mon)
		__field(unsigned short, year)
		__field(unsigned char, alarm_type)
		__string(name, process_name)
	),

	TP_fast_assign(
		__entry->expires = alarm->node.expires.tv64;
		__entry->second = rtc_ktime_to_tm(alarm->node.expires).tm_sec;
		__entry->minute = rtc_ktime_to_tm(alarm->node.expires).tm_min;
		__entry->hour = rtc_ktime_to_tm(alarm->node.expires).tm_hour;
		__entry->day = rtc_ktime_to_tm(alarm->node.expires).tm_mday;
		__entry->mon = rtc_ktime_to_tm(alarm->node.expires).tm_mon;
		__entry->year = rtc_ktime_to_tm(alarm->node.expires).tm_year;
		__entry->alarm_type = alarm->type;
		__assign_str(name, process_name);
	),

	TP_printk("process:%s alarmtimer type:%s expires:%llu "
		  "time: %hu-%u-%u %u:%u:%u",
		  __get_str(name),
		  show_alarm_type((1 << __entry->alarm_type)),
		  __entry->expires,
		  __entry->year + 1900,
		  __entry->mon + 1,
		  __entry->day,
		  __entry->hour,
		  __entry->minute,
		  __entry->second
	)
);

DEFINE_EVENT(alarm_processing, alarmtimer_fired,

	TP_PROTO(struct alarm *alarm, char *process_name),

	TP_ARGS(alarm, process_name)
);

DEFINE_EVENT(alarm_processing, alarmtimer_start,

	TP_PROTO(struct alarm *alarm, char *process_name),

	TP_ARGS(alarm, process_name)
);

DEFINE_EVENT(alarm_processing, alarmtimer_restart,

	TP_PROTO(struct alarm *alarm, char *process_name),

	TP_ARGS(alarm, process_name)
);

DEFINE_EVENT(alarm_processing, alarmtimer_cancel,

	TP_PROTO(struct alarm *alarm, char *process_name),

	TP_ARGS(alarm, process_name)
);

#endif /* _TRACE_ALARMTIMER_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
