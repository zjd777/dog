/* GNU/newlib runtime glue. Application UART output remains __io_putchar. */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

extern int __io_putchar(int ch);
extern char __heap_start, __heap_limit;

void _init(void) {}
void _fini(void) {}

int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _fstat(int file, struct stat *status)
{
    if (file < 0 || file > 2) { errno = EBADF; return -1; }
    status->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    if (file >= 0 && file <= 2) { return 1; }
    errno = EBADF;
    return 0;
}

off_t _lseek(int file, off_t offset, int direction)
{
    (void)file; (void)offset; (void)direction;
    errno = ESPIPE;
    return (off_t)-1;
}

int _read(int file, char *data, int length)
{
    (void)file; (void)data; (void)length;
    errno = ENOSYS;
    return -1;
}

int _write(int file, char *data, int length)
{
    (void)file;
    for (int i = 0; i < length; ++i) {
        __io_putchar((unsigned char)data[i]);
    }
    return length;
}

void *_sbrk(ptrdiff_t increment)
{
    static char *heap = &__heap_start;
    const uintptr_t current = (uintptr_t)heap;
    if ((increment > 0 && (uintptr_t)increment > (uintptr_t)&__heap_limit - current) ||
        (increment < 0 && (uintptr_t)(-(increment + 1)) + 1 > current - (uintptr_t)&__heap_start)) {
        errno = ENOMEM;
        return (void *)-1;
    }
    char *previous = heap;
    heap += increment;
    return previous;
}
