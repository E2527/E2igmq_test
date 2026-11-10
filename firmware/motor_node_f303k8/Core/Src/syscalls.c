/*
 * newlib が使うシステムコールの最小限の代わり(CubeIDE が生成する syscalls.c と同じ役割)
 * printf などは使っていないので、すべて「使えない」と返す
 */
#include <errno.h>
#include <sys/stat.h>

int _close(int file);
int _lseek(int file, int ptr, int dir);
int _read(int file, char *ptr, int len);
int _write(int file, char *ptr, int len);
int _fstat(int file, struct stat *st);
int _isatty(int file);

int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    errno = EBADF;
    return -1;
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}
