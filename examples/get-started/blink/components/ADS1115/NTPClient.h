#include <stdio.h>
#include <time.h>

int app_main(void)
{
    struct tm* ptr;
    time_t t;
    t = time(NULL);
    ptr = localtime(&t);
    printf("%s", asctime(ptr));
    return 0;
}