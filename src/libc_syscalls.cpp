// SPDX-License-Identifier: GPL-3.0-or-later

#include <device.h> /* for ARM CMSIS __BKPT() */
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

extern "C" {

extern int _end;

void _exit(int status);
caddr_t _sbrk(int incr);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, void* buf, size_t cnt);
int _write(int fd, const void* buf, size_t cnt);

void _exit(int status) {
        (void)status;
#ifdef DEBUG
        __BKPT(0);
#endif
        while (1) {
        }
}

caddr_t _sbrk(int incr) {
        static unsigned char* heap = nullptr;
        unsigned char* prev_heap;

        if (heap == nullptr) {
                heap = reinterpret_cast<unsigned char*>(&_end);
        }
        prev_heap = heap;
        heap += incr;
        return reinterpret_cast<caddr_t>(prev_heap);
}

int _close(int fd) {
        (void)fd;
        errno = EBADF;
        return -1;
}

int _lseek(int fd, int ptr, int dir) {
        (void)fd;
        (void)ptr;
        (void)dir;
        errno = EBADF;
        return -1;
}

int _read(int fd, void* buf, size_t cnt) {
        (void)fd;
        (void)buf;
        (void)cnt;
        errno = EBADF;
        return -1;
}

int _write(int fd, const void* buf, size_t cnt) {
        (void)fd;
        (void)buf;
        (void)cnt;
        errno = EBADF;
        return -1;
}

} // extern "C"
