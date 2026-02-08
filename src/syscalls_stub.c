#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/stat.h>

extern char end;
extern char __StackTop;

void *_sbrk(ptrdiff_t increment)
{
  static char *heap_end;
  char *previous_end;

  if (heap_end == 0)
  {
    heap_end = &end;
  }

  if ((heap_end + increment) >= &__StackTop)
  {
    errno = ENOMEM;
    return (void *)-1;
  }

  previous_end = heap_end;
  heap_end += increment;
  return previous_end;
}

int _fstat(int file, struct stat *status)
{
  (void)file;
  if (status != 0)
  {
    status->st_mode = S_IFCHR;
  }
  return 0;
}

int _isatty(int file)
{
  (void)file;
  return 1;
}

int _close(int file)
{
  (void)file;
  return -1;
}

int _lseek(int file, int pointer, int direction)
{
  (void)file;
  (void)pointer;
  (void)direction;
  return 0;
}

int _read(int file, char *buffer, int length)
{
  (void)file;
  (void)buffer;
  (void)length;
  return 0;
}

int _write(int file, const char *buffer, int length)
{
  (void)file;
  (void)buffer;
  return length;
}

int _getpid(void)
{
  return 1;
}

int _kill(int process_id, int signal)
{
  (void)process_id;
  (void)signal;
  errno = EINVAL;
  return -1;
}

void _exit(int status)
{
  (void)status;
  while (1)
  {
  }
}
