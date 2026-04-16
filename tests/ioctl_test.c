#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <time.h>
#include <string.h>
#include "linux/ntsync.h"

int main(void)
{
    int fd = open("/dev/ntsync", O_RDWR);
    if (fd < 0) {
        perror("open /dev/ntsync");
        return 1;
    }

    /* 1. Create a semaphore starting at 0 (unsignaled) */
    struct ntsync_sem_args sem_args = {0};
    sem_args.count = 0; 
    sem_args.max = 10;
    
    int sem_fd = ioctl(fd, NTSYNC_IOC_CREATE_SEM, &sem_args);
    if (sem_fd < 0) {
        perror("ioctl CREATE_SEM");
        close(fd);
        return 1;
    }

    printf("Main: Created semaphore (fd=%d) starting at 0.\n", sem_fd);

    pid_t pid = fork();
    if (pid == 0) {
        /* CHILD PROCESS */
        printf("Child:  Sleeping 2s before releasing...\n");
        sleep(2);
        
        /* The header defines this as taking a pointer to __u32 */
        uint32_t release_count = 1;
        printf("Child:  Releasing semaphore now using NTSYNC_IOC_SEM_RELEASE.\n");
        
        if (ioctl(sem_fd, NTSYNC_IOC_SEM_RELEASE, &release_count) == -1) {
            perror("child: ioctl SEM_RELEASE");
            exit(1);
        }
        exit(0);
    } else {
        /* PARENT PROCESS */
        uint32_t handles[1] = { (uint32_t)sem_fd };
	struct timespec ts;
        struct ntsync_wait_args a;
        memset(&a, 0, sizeof(a));

        /* NTSync expects absolute time in nanoseconds (CLOCK_MONOTONIC) */
        clock_gettime(CLOCK_MONOTONIC, &ts);
        a.timeout = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec + (5ULL * 1000000000ULL);
        a.count = 1;
        a.objs = (uintptr_t)handles;

        printf("Parent: Waiting for semaphore (will block)...\n");
        
        if (ioctl(fd, NTSYNC_IOC_WAIT_ANY, &a) == -1) {
            perror("parent: ioctl WAIT_ANY");
        } else {
            printf("Parent: WOKE UP! Semaphore acquired successfully.\n");
        }

        wait(NULL);
    }

    close(sem_fd);
    close(fd);
    return 0;
}
