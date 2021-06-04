#ifndef __TIMESTAMP_H_
#define __TIMESTAMP_H_

#include <sys/time.h>
#include <time.h>

long getCurrenTime_s()
{
	time_t sectime;
	sectime = time( NULL );
	return sectime;
}

long getCurrentTime_ms()
{
	struct timeval tv;
	gettimeofday( &tv, NULL );
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

long getCurrentTime_us()
{
	struct timeval tv;
	gettimeofday( &tv, NULL );
	return tv.tv_sec * 1000000 + tv.tv_usec;
}

#endif
