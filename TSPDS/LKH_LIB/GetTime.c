#define HAVE_GETRUSAGE
/* Undefine if you don't have the getrusage function */
/* #undef HAVE_GETRUSAGE */

/*
 * The GetTime function is used to measure execution time.
 *
 * The function is called before and after the code to be 
 * measured. The difference between the second and the
 * first call gives the number of seconds spent in executing
 * the code.
 *
 * If the system call getrusage() is supported, the difference 
 * gives the user time used; otherwise, the accounted real time.
 */

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#include <sys/resource.h>
#endif

#ifdef _WIN32
static double windows_get_time(void)
{
    FILETIME createTime, exitTime, kernelTime, userTime;
    if (GetProcessTimes(GetCurrentProcess(), &createTime, &exitTime, &kernelTime, &userTime)) {
        ULARGE_INTEGER userSystemTime;
        userSystemTime.LowPart = userTime.dwLowDateTime;
        userSystemTime.HighPart = userTime.dwHighDateTime;
        return (double)(userSystemTime.QuadPart) * 0.0000001;
    }
    return 0;
}
#endif

double GetTime(void)
{
#ifdef _WIN32
    return windows_get_time();
#else
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1000000.0;
#endif
}