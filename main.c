
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[])
{

    printf("\nHello from ble-gate\n\n");

    fprintf(stderr, "Enabled: %s", argv[0]);
    fprintf(stderr, "FTP: %s", argv[1]);
    fprintf(stderr, "Timeout: %s", argv[2]);
    return 0;

}