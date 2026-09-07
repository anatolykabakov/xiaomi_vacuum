/* Объявления glibc>=2.30, которых нет в pthread.h 2.19 (реализация — glibc_compat.o). Только для C++. */
#pragma once
#ifdef __cplusplus
#include <pthread.h>
#include <time.h>
extern "C" {
int pthread_cond_clockwait(pthread_cond_t*, pthread_mutex_t*, clockid_t, const struct timespec*);
int pthread_mutex_clocklock(pthread_mutex_t*, clockid_t, const struct timespec*);
}
#endif
