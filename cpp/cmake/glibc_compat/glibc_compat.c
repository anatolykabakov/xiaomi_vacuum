/* Совместимость gcc-11 libstdc++/conan-libs с glibc 2.19 робота: недостающие символы. */
#define _GNU_SOURCE
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <fcntl.h>
char __libc_single_threaded = 0;
void* __explicit_bzero_chk(void* s, size_t n, size_t dstlen) { (void)dstlen; volatile unsigned char* p = s; while (n--) *p++ = 0; return s; }
ssize_t getrandom(void* buf, size_t n, unsigned flags) {
#ifdef __NR_getrandom
  long r = syscall(__NR_getrandom, buf, n, flags); if (r >= 0 || errno != ENOSYS) return r;
#else
  (void)flags;
#endif
  int fd = open("/dev/urandom", O_RDONLY | O_CLOEXEC); if (fd < 0) return -1;
  ssize_t got = read(fd, buf, n); close(fd); return got;
}
int pthread_cond_clockwait(pthread_cond_t* c, pthread_mutex_t* m, clockid_t clk, const struct timespec* abst) {
  if (clk == CLOCK_REALTIME) return pthread_cond_timedwait(c, m, abst);
  struct timespec now_c, now_r, t; clock_gettime(clk, &now_c); clock_gettime(CLOCK_REALTIME, &now_r);
  t.tv_sec = now_r.tv_sec + (abst->tv_sec - now_c.tv_sec); t.tv_nsec = now_r.tv_nsec + (abst->tv_nsec - now_c.tv_nsec);
  while (t.tv_nsec >= 1000000000L) { t.tv_nsec -= 1000000000L; t.tv_sec++; } while (t.tv_nsec < 0) { t.tv_nsec += 1000000000L; t.tv_sec--; }
  return pthread_cond_timedwait(c, m, &t);
}
int pthread_mutex_clocklock(pthread_mutex_t* m, clockid_t clk, const struct timespec* abst) {
  if (clk == CLOCK_REALTIME) return pthread_mutex_timedlock(m, abst);
  struct timespec now_c, now_r, t; clock_gettime(clk, &now_c); clock_gettime(CLOCK_REALTIME, &now_r);
  t.tv_sec = now_r.tv_sec + (abst->tv_sec - now_c.tv_sec); t.tv_nsec = now_r.tv_nsec + (abst->tv_nsec - now_c.tv_nsec);
  while (t.tv_nsec >= 1000000000L) { t.tv_nsec -= 1000000000L; t.tv_sec++; } while (t.tv_nsec < 0) { t.tv_nsec += 1000000000L; t.tv_sec--; }
  return pthread_mutex_timedlock(m, &t);
}
