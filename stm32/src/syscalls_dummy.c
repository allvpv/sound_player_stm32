#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>

void _exit(int status) {
  (void)status;
  for (;;);
}

int _kill(pid_t pid, int sig) {
  (void)pid;
  (void)sig;
  errno = EPERM;
  return -1;
}

pid_t _getpid(void) {
  return 0;
}

int _close(int fd) {
  (void)fd;
  errno = 0;
  return 0;
}

long _read(int fd, char *ptr, long len) {
  (void)fd;
  (void)ptr;
  (void)len;
  errno = EBADF;
  return -1;
}

long _write(int fd, char const *ptr, long len) {
  (void)fd;
  (void)ptr;
  (void)len;
  errno = EBADF;
  return -1;
}

long _lseek(int fd, long offset, int whence) {
  (void)fd;
  (void)offset;
  (void)whence;
  errno = EBADF;
  return -1;
}

int _fstat(int fd, struct stat *st) {
  (void)fd;
  (void)st;
  errno = EBADF;
  return -1;
}

int _isatty(int fd) {
  (void)fd;
  errno = EBADF;
  return 0;
}
