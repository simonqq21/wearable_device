#include <stdio.h>
#include <string.h>

#define PREFIX "imu_raw_"
#define FORMAT PREFIX "%06d.bin"

int main()
{
    char str1[20];
    // printf(FORMAT "\n", 67);
    sprintf(str1, FORMAT, 67);
    printf("str1 = %s\n", str1);
    int res = strncmp(PREFIX, str1, strlen(PREFIX));
    printf("res = %d\n", res);
    int res2 = strcmp(PREFIX, str1);
    printf("res2 = %d\n", res2);
    int res3 = strcmp(str1, PREFIX);
    printf("res3 = %d\n", res3);
    return 0;
}