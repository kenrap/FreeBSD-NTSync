/*
 * ntsync_test.c
 *
 * Test harness for the ntsync driver using the real ntsync.h ABI.
 *
 * Compile:
 *   cc -O2 -Wall -o ntsync_test_with_header ntsync_test_with_header.c
 * or, if ntsync.h is in another directory:
 *   cc -O2 -Wall -I/usr/local/include -o ntsync_test ntsync_test.c
 *
 * Run:
 *   ./ntsync_test /dev/ntsync
 *
 * The program uses NTSYNC_IOC_WAIT_ANY by default; change CMD to test other ioctls.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <inttypes.h>

#include "ntsync.h"

int main(int argc, char **argv)
{
    const char *devpath;
    int fd;
    int rc;

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <device-path> [ioctl-cmd]\n", argv[0]);
        return 2;
    }
    devpath = argv[1];

    /* Default to WAIT_ANY; override by passing numeric ioctl on command line */
    unsigned long cmd = NTSYNC_IOC_WAIT_ANY;
    if (argc >= 3) {
        if (strstr(argv[2], "0x") == argv[2] || strstr(argv[2], "0X") == argv[2])
            cmd = strtoul(argv[2], NULL, 16);
        else
            cmd = strtoul(argv[2], NULL, 0);
    }

    fd = open(devpath, O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    printf("Opened %s -> fd=%d, ioctl cmd=0x%lx\n", devpath, fd, cmd);

    /* Phase 0: Create a real object to get a valid FD */
	struct ntsync_sem_args s_args = { .count = 1, .max = 1 };
	int sem_fd = ioctl(fd, NTSYNC_IOC_CREATE_SEM, &s_args);
	if (sem_fd < 0) {
	    perror("ioctl(NTSYNC_IOC_CREATE_SEM)");
	return 1;
	}

    /* Prepare a small array in userland that the kernel will copyin from */
    uint64_t objs_arr[NTSYNC_MAX_WAIT_COUNT];
    objs_arr[0] = (uint64_t)sem_fd; /* Use the real FD */

    /* Build the user struct with embedded pointer to objs_arr */
    struct ntsync_wait_args args;
    memset(&args, 0, sizeof(args));
    args.timeout = 0;           /* immediate */
    args.objs = (uint64_t)(uintptr_t)objs_arr;
    args.count = 1;
    args.index = (uint32_t)-1;
    args.flags = 0;

    errno = 0;
    rc = ioctl(fd, (unsigned long)cmd, &args);
    if (rc == -1) {
        int e = errno;
        printf("  ioctl returned -1 errno=%d (%s)\n", e, strerror(e));
    } else {
        printf("  ioctl returned %d\n", rc);
    }
    printf("  after ioctl: args.index=%u args.count=%u\n", args.index, args.count);

    /* Test 2: invalid embedded pointer (small integer) */
    struct ntsync_wait_args bad_args;
    memcpy(&bad_args, &args, sizeof(bad_args));
    bad_args.objs = 1; /* intentionally invalid user pointer */

    printf("\nTest 2: invalid embedded pointer (should produce EFAULT)\n");
    printf("  user struct @%p, bad_args.objs (user ptr) = %p\n",
           (void *)&bad_args, (void *)(uintptr_t)bad_args.objs);

    errno = 0;
    rc = ioctl(fd, (unsigned long)cmd, &bad_args);
    if (rc == -1) {
        int e = errno;
        printf("  ioctl returned -1 errno=%d (%s)\n", e, strerror(e));
        if (e == EFAULT)
            printf("  -> kernel correctly returned EFAULT for invalid embedded pointer\n");
    } else {
        printf("  ioctl returned %d (unexpected)\n", rc);
    }

    close(sem_fd);
    close(fd);
    return 0;
}
