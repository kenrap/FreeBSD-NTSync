#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <string.h>
#include <errno.h>
#include "ntsync.h"

int main(void)
{
    int fd = open("/dev/ntsync", O_RDWR);
    if (fd < 0) { perror("open /dev/ntsync"); return 1; }

    /* 1) Create a semaphore (example)
     *
     * Try a stack allocation first and print the full pointer value so the
     * kernel-side debug can be compared. If the stack allocation fails with
     * EFAULT, retry with a heap allocation to rule out stack/scope issues.
     */
    int rc;
    uint32_t created_id = 0;

    struct ntsync_sem_args sem_stack = { .count = 1, .max = 1000 };
    printf("user: &sem_stack=%p (0x%016jx)\n",
           (void *)&sem_stack, (uintmax_t)(uint64_t)(uintptr_t)&sem_stack);

    /* First try the normal libc ioctl and then a direct syscall to compare. */
    rc = ioctl(fd, NTSYNC_IOC_CREATE_SEM, &sem_stack);
    if (rc < 0) {
        int e = errno;
        printf("user: CREATE_SEM (stack ioctl) failed rc=%d errno=%d (%s)\n", rc, e, strerror(e));

        /* Try the raw syscall path to rule out a libc wrapper issue. */
        long rc_sys = syscall(SYS_ioctl, fd, NTSYNC_IOC_CREATE_SEM, (void *)(uintptr_t)&sem_stack);
        printf("user: CREATE_SEM (stack syscall) rc=%ld errno=%d (%s)\n",
               rc_sys, errno, strerror(errno));

        /* Retry with heap allocation to rule out stack pointer issues */
        struct ntsync_sem_args *sem_heap = malloc(sizeof(*sem_heap));
        if (sem_heap == NULL) {
            perror("malloc sem_heap");
            close(fd);
            return 1;
        }
        sem_heap->count = 0;
        sem_heap->max = 1000;
        printf("user: sem_heap=%p (0x%016jx)\n",
               (void *)sem_heap, (uintmax_t)(uint64_t)(uintptr_t)sem_heap);

        rc = ioctl(fd, NTSYNC_IOC_CREATE_SEM, sem_heap);
        if (rc < 0) {
            int e2 = errno;
            printf("user: CREATE_SEM (heap ioctl) failed rc=%d errno=%d (%s)\n", rc, e2, strerror(e2));
            long rc_sys2 = syscall(SYS_ioctl, fd, NTSYNC_IOC_CREATE_SEM, (void *)(uintptr_t)sem_heap);
            printf("user: CREATE_SEM (heap syscall) rc=%ld errno=%d (%s)\n",
                   rc_sys2, errno, strerror(errno));
            free(sem_heap);
            close(fd);
            return 1;
        }
        /* success with heap via ioctl */
        created_id = (uint32_t)rc;
        printf("create ioctl (heap) returned %d -> using id=%u\n", rc, created_id);
        free(sem_heap);
    } else {
        /* success with stack via ioctl */
        created_id = (uint32_t)rc;
        printf("create ioctl (stack) returned %d -> using id=%u\n", rc, created_id);
    }

    /* If the driver instead writes the id into the struct, check fields:
     * (uncomment if your driver writes back into sem.count or similar)
     */
    /* printf("sem.count=%u sem.max=%u\n", sem.count, sem.max); */


    /* 2) Prepare objs array in userland and point a.objs at it.
     *
     * Use a heap allocation for objs to avoid any stack-scope issues and
     * print the full pointer values so they can be compared to kernel logs.
     */
    uint32_t *objs = malloc(sizeof(*objs));
    if (objs == NULL) {
        perror("malloc objs");
        close(fd);
        return 1;
    }
    objs[0] = created_id;

    struct ntsync_wait_args a;
    memset(&a, 0, sizeof(a));
    a.objs = (uint64_t)(uintptr_t)objs; /* user pointer to array of ids */
    a.count = 1;
    a.timeout = 2000ULL; /* small-ms heuristic: 2000 ms */
    a.flags = 0;

    printf("user: objs=%p (0x%016jx)\n", (void *)objs, (uintmax_t)(uint64_t)(uintptr_t)objs);
    printf("user: a.objs=(uint64_t)(uintptr_t)objs -> 0x%016jx\n",
           (uintmax_t)(uint64_t)(uintptr_t)objs);

    printf("user: sending timeout(ms) = %ju (0x%016jx) objs_ptr=%p id=%u\n",
           (uintmax_t)a.timeout, (uintmax_t)a.timeout, (void *)(uintptr_t)a.objs, objs[0]);

    rc = ioctl(fd, NTSYNC_IOC_WAIT_ANY, &a);
    if (rc < 0) {
        int e = errno;
        printf("ioctl WAIT_ANY returned %d errno=%d (%s)\n", rc, e, strerror(e));
    } else {
        printf("ioctl WAIT_ANY returned %d\n", rc);
    }

    free(objs);

    close(fd);
    return 0;
}
