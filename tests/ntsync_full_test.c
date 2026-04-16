/*
 * ntsync_full_test.c
 *
 * Comprehensive test harness for ntsync driver using the real ntsync.h ABI.
 *
 * Compile:
 *   cc -O2 -Wall -I/usr/local/include -o ntsync_full_test ntsync_full_test.c
 *
 * Run:
 *   sudo ./ntsync_full_test /dev/ntsync
 *
 * Fixed bugs vs original:
 *   - CREATE_* ioctls return a per-object fd; subsequent object ioctls must
 *     use that fd, not the control device fd.
 *   - WAIT_ANY/WAIT_ALL objs array must contain real object fds, not
 *     arbitrary integers.
 *   - All object fds are now properly closed.
 *   - EVENT_SET/RESET/PULSE return values (previous signaled state) are now
 *     read and printed.
 *   - Added EOVERFLOW test for semaphore release past max.
 *   - Added WAIT_ANY timeout test (non-signaled object, short timeout).
 *   - Added multi-object WAIT_ANY test (two sems, one signaled).
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

#include "linux/ntsync.h"

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, fmt, ...) do {                                      \
    if (cond) {                                                         \
        printf("  PASS: " fmt "\n", ##__VA_ARGS__);                     \
        g_pass++;                                                       \
    } else {                                                            \
        fprintf(stderr, "  FAIL: " fmt "\n", ##__VA_ARGS__);           \
        g_fail++;                                                       \
    }                                                                   \
} while (0)

/*
 * Wrapper around ioctl that prints the result and returns the raw rc.
 * For control-device ioctls (CREATE_*) rc is the new object fd on success.
 * For object ioctls rc is 0 on success, -1 on error.
 */
static int do_ioctl(int fd, unsigned long cmd, void *arg, const char *label)
{
    int rc;
    errno = 0;
    rc = ioctl(fd, cmd, arg);
    if (rc == -1)
        fprintf(stderr, "  %s: ioctl -> -1 errno=%d (%s)\n",
                label, errno, strerror(errno));
    else
        printf("  %s: ioctl -> %d\n", label, rc);
    return rc;
}

/* Close an fd and zero it so we don't double-close on cleanup. */
static void safe_close(int *fd)
{
    if (*fd >= 0) {
        close(*fd);
        *fd = -1;
    }
}

/* ------------------------------------------------------------------ */
/* Test sections                                                        */
/* ------------------------------------------------------------------ */

static int test_semaphore(int dev_fd)
{
    int sem_fd = -1;
    int rc;

    printf("\n========== SEMAPHORE TESTS ==========\n");

    /* --- Create --- */
    printf("\n-- Create semaphore (count=2 max=10) --\n");
    {
        struct ntsync_sem_args args = { .count = 2, .max = 10 };
        /*
         * FIX: CREATE_SEM returns a new object fd on success.
         * Save it; all subsequent sem operations use sem_fd, not dev_fd.
         */
        sem_fd = do_ioctl(dev_fd, NTSYNC_IOC_CREATE_SEM, &args, "CREATE_SEM");
        CHECK(sem_fd >= 0, "CREATE_SEM returned valid fd=%d", sem_fd);
        if (sem_fd < 0) return -1;
    }

    /* --- Read initial state --- */
    printf("\n-- Read initial semaphore state --\n");
    {
        struct ntsync_sem_args rd = { 0 };
        rc = do_ioctl(sem_fd, NTSYNC_IOC_SEM_READ, &rd, "SEM_READ");
        CHECK(rc == 0, "SEM_READ succeeded");
        CHECK(rd.count == 2, "initial count=2 (got %u)", rd.count);
        CHECK(rd.max   == 10, "initial max=10 (got %u)",  rd.max);
    }

    /* --- Release by 3 (2 -> 5) --- */
    printf("\n-- Release semaphore by 3 (count 2 -> 5) --\n");
    {
        __u32 prev = 0;
        __u32 add  = 3;
        /*
         * SEM_RELEASE takes/returns a __u32 through the same pointer:
         * on entry it is the release amount; on return it is the previous count.
         */
        prev = add; /* driver reads this as the release amount */
        rc = do_ioctl(sem_fd, NTSYNC_IOC_SEM_RELEASE, &prev, "SEM_RELEASE(+3)");
        CHECK(rc == 0,   "SEM_RELEASE succeeded");
        CHECK(prev == 2, "previous count was 2 (got %u)", prev);

        struct ntsync_sem_args rd = { 0 };
        do_ioctl(sem_fd, NTSYNC_IOC_SEM_READ, &rd, "SEM_READ after release");
        CHECK(rd.count == 5, "count is now 5 (got %u)", rd.count);
    }

    /* --- Overflow: release by 6 should push count to 11 > max=10 -> EOVERFLOW --- */
    printf("\n-- Overflow: release by 6 (5+6=11 > max=10, expect EOVERFLOW) --\n");
    {
        __u32 add = 6;
        rc = do_ioctl(sem_fd, NTSYNC_IOC_SEM_RELEASE, &add, "SEM_RELEASE(+6 overflow)");
        CHECK(rc == -1 && errno == EOVERFLOW,
              "EOVERFLOW on release past max (rc=%d errno=%d)", rc, errno);
    }

    /* --- WAIT_ANY: semaphore is signaled (count=5), should succeed immediately --- */
    printf("\n-- WAIT_ANY on signaled semaphore (expect immediate success) --\n");
    {
        /*
         * FIX: objs array must contain the real object fd, not a placeholder.
         */
        uint32_t objs[1] = { (uint32_t)sem_fd };
        struct ntsync_wait_args wa = {
            .timeout = 0,   /* immediate */
            .objs    = (uint64_t)(uintptr_t)objs,
            .count   = 1,
            .index   = (uint32_t)-1,
        };
        rc = do_ioctl(dev_fd, NTSYNC_IOC_WAIT_ANY, &wa, "WAIT_ANY(sem signaled)");
        CHECK(rc == 0,           "WAIT_ANY succeeded");
        CHECK(wa.index == 0,     "signaled index is 0 (got %u)", wa.index);

        /* Verify count was consumed: 5 -> 4 */
        struct ntsync_sem_args rd = { 0 };
        do_ioctl(sem_fd, NTSYNC_IOC_SEM_READ, &rd, "SEM_READ after WAIT_ANY");
        CHECK(rd.count == 4, "count decremented to 4 (got %u)", rd.count);
    }

    /* --- WAIT_ANY: drain remaining 4 tokens then expect ETIMEDOUT --- */
    printf("\n-- Drain semaphore to 0, then WAIT_ANY should time out --\n");
    {
        /* Drain the remaining 4 tokens */
        for (int i = 0; i < 4; i++) {
            uint32_t objs[1] = { (uint32_t)sem_fd };
            struct ntsync_wait_args wa = {
                .timeout = 0,
                .objs    = (uint64_t)(uintptr_t)objs,
                .count   = 1,
                .index   = (uint32_t)-1,
            };
            rc = do_ioctl(dev_fd, NTSYNC_IOC_WAIT_ANY, &wa,
                          "WAIT_ANY(drain)");
            CHECK(rc == 0, "drain step %d succeeded", i + 1);
        }

        /* Now count=0; a zero-timeout wait should return ETIMEDOUT */
        uint32_t objs[1] = { (uint32_t)sem_fd };
        struct ntsync_wait_args wa = {
            .timeout = 0,
            .objs    = (uint64_t)(uintptr_t)objs,
            .count   = 1,
            .index   = (uint32_t)-1,
        };
        rc = do_ioctl(dev_fd, NTSYNC_IOC_WAIT_ANY, &wa,
                      "WAIT_ANY(expect ETIMEDOUT)");
        CHECK(rc == -1 && errno == ETIMEDOUT,
              "ETIMEDOUT on unsignaled sem (rc=%d errno=%d)", rc, errno);
    }

    /* --- Multi-object WAIT_ANY: two sems, only second is signaled --- */
    printf("\n-- Multi-object WAIT_ANY: two sems, only index 1 is signaled --\n");
    {
        int sem2_fd = -1, sem3_fd = -1;

        /* sem2: count=0 (not signaled) */
        struct ntsync_sem_args a2 = { .count = 0, .max = 5 };
        sem2_fd = do_ioctl(dev_fd, NTSYNC_IOC_CREATE_SEM, &a2, "CREATE_SEM(count=0)");
        CHECK(sem2_fd >= 0, "second sem created fd=%d", sem2_fd);

        /* sem3: count=1 (signaled) */
        struct ntsync_sem_args a3 = { .count = 1, .max = 5 };
        sem3_fd = do_ioctl(dev_fd, NTSYNC_IOC_CREATE_SEM, &a3, "CREATE_SEM(count=1)");
        CHECK(sem3_fd >= 0, "third sem created fd=%d", sem3_fd);

        if (sem2_fd >= 0 && sem3_fd >= 0) {
            uint32_t objs[2] = { (uint32_t)sem2_fd, (uint32_t)sem3_fd };
            struct ntsync_wait_args wa = {
                .timeout = 0,
                .objs    = (uint64_t)(uintptr_t)objs,
                .count   = 2,
                .index   = (uint32_t)-1,
            };
            rc = do_ioctl(dev_fd, NTSYNC_IOC_WAIT_ANY, &wa,
                          "WAIT_ANY(2 objs, idx1 signaled)");
            CHECK(rc == 0,       "WAIT_ANY succeeded");
            CHECK(wa.index == 1, "signaled index is 1 (got %u)", wa.index);
        }

        safe_close(&sem2_fd);
        safe_close(&sem3_fd);
    }

    safe_close(&sem_fd);
    return 0;
}

static int test_mutex(int dev_fd)
{
    int mutex_fd = -1;
    int rc;
    uint32_t owner = (uint32_t)getpid();

    printf("\n========== MUTEX TESTS ==========\n");

    /* --- Create --- */
    printf("\n-- Create mutex (owner=pid count=1) --\n");
    {
        struct ntsync_mutex_args args = { .owner = owner, .count = 1 };
        /*
         * FIX: save the returned object fd.
         */
        mutex_fd = do_ioctl(dev_fd, NTSYNC_IOC_CREATE_MUTEX, &args, "CREATE_MUTEX");
        CHECK(mutex_fd >= 0, "CREATE_MUTEX returned valid fd=%d", mutex_fd);
        if (mutex_fd < 0) return -1;
    }

    /* --- Read initial state --- */
    printf("\n-- Read initial mutex state --\n");
    {
        struct ntsync_mutex_args rd = { 0 };
        rc = do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd, "MUTEX_READ");
        CHECK(rc == 0,          "MUTEX_READ succeeded");
        CHECK(rd.owner == owner, "owner is pid (got %u)", rd.owner);
        CHECK(rd.count == 1,     "count=1 (got %u)", rd.count);
    }

    /* --- Unlock --- */
    printf("\n-- Unlock mutex (owner=pid) --\n");
    {
        struct ntsync_mutex_args ul = { .owner = owner, .count = 0 };
        rc = do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_UNLOCK, &ul, "MUTEX_UNLOCK");
        CHECK(rc == 0, "MUTEX_UNLOCK succeeded");

        struct ntsync_mutex_args rd = { 0 };
        do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd, "MUTEX_READ after unlock");
        CHECK(rd.owner == 0, "owner cleared (got %u)", rd.owner);
        CHECK(rd.count == 0, "count=0 (got %u)", rd.count);
    }

    /* --- WAIT_ANY on the now-unowned (signaled) mutex --- */
    printf("\n-- WAIT_ANY on unowned mutex (expect immediate success) --\n");
    {
        uint32_t objs[1] = { (uint32_t)mutex_fd };
        struct ntsync_wait_args wa = {
            .timeout = 0,
            .objs    = (uint64_t)(uintptr_t)objs,
            .count   = 1,
            .index   = (uint32_t)-1,
            .owner   = owner,
        };
        rc = do_ioctl(dev_fd, NTSYNC_IOC_WAIT_ANY, &wa, "WAIT_ANY(mutex)");
        CHECK(rc == 0,       "WAIT_ANY succeeded");
        CHECK(wa.index == 0, "signaled index is 0 (got %u)", wa.index);

        /* Verify mutex is now owned by us */
        struct ntsync_mutex_args rd = { 0 };
        do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd, "MUTEX_READ after WAIT_ANY");
        CHECK(rd.owner == owner, "mutex now owned by pid (got %u)", rd.owner);
        CHECK(rd.count == 1,     "count=1 (got %u)", rd.count);
    }

    /* --- MUTEX_KILL: abandon mutex, leaves it in ownerdead state --- */
    printf("\n-- Kill mutex (owner=pid) --\n");
    {
        __u32 kill_owner = owner;
        rc = do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_KILL, &kill_owner, "MUTEX_KILL");
        CHECK(rc == 0, "MUTEX_KILL succeeded");

        struct ntsync_mutex_args rd = { 0 };
        do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd, "MUTEX_READ after kill");
        CHECK(rd.owner == 0, "owner cleared after kill (got %u)", rd.owner);
        CHECK(rd.count == 0, "count=0 after kill (got %u)",        rd.count);
    }

    safe_close(&mutex_fd);
    return 0;
}

static int test_event(int dev_fd)
{
    int ev_fd = -1;
    int rc;

    printf("\n========== EVENT TESTS ==========\n");

    /* --- Create manual-reset event, initially not signaled --- */
    printf("\n-- Create manual-reset event (signaled=0) --\n");
    {
        struct ntsync_event_args args = { .manual = 1, .signaled = 0 };
        /*
         * FIX: save the returned object fd.
         */
        ev_fd = do_ioctl(dev_fd, NTSYNC_IOC_CREATE_EVENT, &args, "CREATE_EVENT");
        CHECK(ev_fd >= 0, "CREATE_EVENT returned valid fd=%d", ev_fd);
        if (ev_fd < 0) return -1;
    }

    /* --- Read initial state --- */
    printf("\n-- Read initial event state --\n");
    {
        struct ntsync_event_args rd = { 0 };
        rc = do_ioctl(ev_fd, NTSYNC_IOC_EVENT_READ, &rd, "EVENT_READ");
        CHECK(rc == 0,         "EVENT_READ succeeded");
        CHECK(rd.manual == 1,  "manual=1 (got %u)",   rd.manual);
        CHECK(rd.signaled == 0,"signaled=0 (got %u)", rd.signaled);
    }

    /* --- Set event; kernel writes back previous signaled state --- */
    printf("\n-- Set event (expect previous state=0) --\n");
    {
        __u32 prev = 0;
        rc = do_ioctl(ev_fd, NTSYNC_IOC_EVENT_SET, &prev, "EVENT_SET");
        CHECK(rc == 0,     "EVENT_SET succeeded");
        /*
         * FIX: EVENT_SET is _IOR — kernel writes the previous signaled value
         * back into the buffer.  Check it rather than ignoring it.
         */
        CHECK(prev == 0, "previous signaled was 0 (got %u)", prev);

        struct ntsync_event_args rd = { 0 };
        do_ioctl(ev_fd, NTSYNC_IOC_EVENT_READ, &rd, "EVENT_READ after set");
        CHECK(rd.signaled == 1, "signaled=1 after set (got %u)", rd.signaled);
    }

    /* --- Reset event --- */
    printf("\n-- Reset event (expect previous state=1) --\n");
    {
        __u32 prev = 0;
        rc = do_ioctl(ev_fd, NTSYNC_IOC_EVENT_RESET, &prev, "EVENT_RESET");
        CHECK(rc == 0,     "EVENT_RESET succeeded");
        CHECK(prev == 1, "previous signaled was 1 (got %u)", prev);

        struct ntsync_event_args rd = { 0 };
        do_ioctl(ev_fd, NTSYNC_IOC_EVENT_READ, &rd, "EVENT_READ after reset");
        CHECK(rd.signaled == 0, "signaled=0 after reset (got %u)", rd.signaled);
    }

    /* --- Pulse event: atomically set then reset (no waiter, so no effect) --- */
    printf("\n-- Pulse event (no waiter; event should return to unsignaled) --\n");
    {
        __u32 prev = 0;
        rc = do_ioctl(ev_fd, NTSYNC_IOC_EVENT_PULSE, &prev, "EVENT_PULSE");
        CHECK(rc == 0,     "EVENT_PULSE succeeded");
        CHECK(prev == 0, "previous signaled was 0 (got %u)", prev);

        struct ntsync_event_args rd = { 0 };
        do_ioctl(ev_fd, NTSYNC_IOC_EVENT_READ, &rd, "EVENT_READ after pulse");
        CHECK(rd.signaled == 0, "signaled=0 after pulse (no waiter) (got %u)", rd.signaled);
    }

    /* --- WAIT_ANY on unsignaled event at zero timeout should ETIMEDOUT --- */
    printf("\n-- WAIT_ANY on unsignaled event (expect ETIMEDOUT) --\n");
    {
        uint32_t objs[1] = { (uint32_t)ev_fd };
        struct ntsync_wait_args wa = {
            .timeout = 0,
            .objs    = (uint64_t)(uintptr_t)objs,
            .count   = 1,
            .index   = (uint32_t)-1,
        };
        rc = do_ioctl(dev_fd, NTSYNC_IOC_WAIT_ANY, &wa, "WAIT_ANY(event unsignaled)");
        CHECK(rc == -1 && errno == ETIMEDOUT,
              "ETIMEDOUT on unsignaled event (rc=%d errno=%d)", rc, errno);
    }

    /* --- Set event, then WAIT_ANY should succeed immediately --- */
    printf("\n-- Set event then WAIT_ANY (expect immediate success) --\n");
    {
        __u32 prev = 0;
        do_ioctl(ev_fd, NTSYNC_IOC_EVENT_SET, &prev, "EVENT_SET(pre-wait)");

        uint32_t objs[1] = { (uint32_t)ev_fd };
        struct ntsync_wait_args wa = {
            .timeout = 0,
            .objs    = (uint64_t)(uintptr_t)objs,
            .count   = 1,
            .index   = (uint32_t)-1,
        };
        rc = do_ioctl(dev_fd, NTSYNC_IOC_WAIT_ANY, &wa, "WAIT_ANY(event signaled)");
        CHECK(rc == 0,       "WAIT_ANY succeeded on signaled event");
        CHECK(wa.index == 0, "signaled index is 0 (got %u)", wa.index);

        /*
         * For a manual-reset event, the event remains signaled after a wait;
         * the caller must reset it explicitly.
         */
        struct ntsync_event_args rd = { 0 };
        do_ioctl(ev_fd, NTSYNC_IOC_EVENT_READ, &rd, "EVENT_READ after WAIT_ANY");
        CHECK(rd.signaled == 1,
              "manual-reset event stays signaled after WAIT_ANY (got %u)",
              rd.signaled);
    }

    /* --- Auto-reset event: should clear signaled after a successful wait --- */
    printf("\n-- Auto-reset event (manual=0): signaled cleared after WAIT_ANY --\n");
    {
        int autoev_fd = -1;
        struct ntsync_event_args args = { .manual = 0, .signaled = 1 };
        autoev_fd = do_ioctl(dev_fd, NTSYNC_IOC_CREATE_EVENT, &args, "CREATE_EVENT(auto,signaled=1)");
        CHECK(autoev_fd >= 0, "auto-reset event created fd=%d", autoev_fd);

        if (autoev_fd >= 0) {
            uint32_t objs[1] = { (uint32_t)autoev_fd };
            struct ntsync_wait_args wa = {
                .timeout = 0,
                .objs    = (uint64_t)(uintptr_t)objs,
                .count   = 1,
                .index   = (uint32_t)-1,
            };
            rc = do_ioctl(dev_fd, NTSYNC_IOC_WAIT_ANY, &wa, "WAIT_ANY(auto-reset)");
            CHECK(rc == 0,       "WAIT_ANY succeeded");
            CHECK(wa.index == 0, "signaled index is 0 (got %u)", wa.index);

            struct ntsync_event_args rd = { 0 };
            do_ioctl(autoev_fd, NTSYNC_IOC_EVENT_READ, &rd, "EVENT_READ after WAIT_ANY");
            CHECK(rd.signaled == 0,
                  "auto-reset event is now unsignaled (got %u)", rd.signaled);

            safe_close(&autoev_fd);
        }
    }

    safe_close(&ev_fd);
    return 0;
}

static int test_wait_all(int dev_fd)
{
    int sem_fd = -1, ev_fd = -1;
    int rc;

    printf("\n========== WAIT_ALL TESTS ==========\n");

    /* Two objects, both signaled: WAIT_ALL should succeed immediately. */
    printf("\n-- WAIT_ALL: two signaled objects (expect success) --\n");
    {
        struct ntsync_sem_args sa = { .count = 1, .max = 5 };
        sem_fd = do_ioctl(dev_fd, NTSYNC_IOC_CREATE_SEM, &sa, "CREATE_SEM(count=1)");
        CHECK(sem_fd >= 0, "sem created fd=%d", sem_fd);

        struct ntsync_event_args ea = { .manual = 1, .signaled = 1 };
        ev_fd = do_ioctl(dev_fd, NTSYNC_IOC_CREATE_EVENT, &ea, "CREATE_EVENT(signaled=1)");
        CHECK(ev_fd >= 0, "event created fd=%d", ev_fd);

        if (sem_fd >= 0 && ev_fd >= 0) {
            uint32_t objs[2] = { (uint32_t)sem_fd, (uint32_t)ev_fd };
            struct ntsync_wait_args wa = {
                .timeout = 0,
                .objs    = (uint64_t)(uintptr_t)objs,
                .count   = 2,
                .index   = (uint32_t)-1,
            };
            rc = do_ioctl(dev_fd, NTSYNC_IOC_WAIT_ALL, &wa, "WAIT_ALL(both signaled)");
            CHECK(rc == 0, "WAIT_ALL succeeded (rc=%d)", rc);
        }
    }

    /* One of two objects not signaled: WAIT_ALL should time out. */
    printf("\n-- WAIT_ALL: one unsignaled object (expect ETIMEDOUT) --\n");
    {
        if (sem_fd >= 0 && ev_fd >= 0) {
            /*
             * sem_fd was consumed by the previous WAIT_ALL (count 1->0);
             * event remains signaled (manual-reset).  Attempting WAIT_ALL
             * again should time out because the sem is now unsignaled.
             */
            uint32_t objs[2] = { (uint32_t)sem_fd, (uint32_t)ev_fd };
            struct ntsync_wait_args wa = {
                .timeout = 0,
                .objs    = (uint64_t)(uintptr_t)objs,
                .count   = 2,
                .index   = (uint32_t)-1,
            };
            rc = do_ioctl(dev_fd, NTSYNC_IOC_WAIT_ALL, &wa, "WAIT_ALL(sem unsignaled)");
            CHECK(rc == -1 && errno == ETIMEDOUT,
                  "ETIMEDOUT when one object unsignaled (rc=%d errno=%d)", rc, errno);
        }
    }

    safe_close(&sem_fd);
    safe_close(&ev_fd);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Main                                                                 */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <device-path>\n", argv[0]);
        return 2;
    }

    int dev_fd = open(argv[1], O_RDWR);
    if (dev_fd < 0) {
        perror("open");
        return 1;
    }
    printf("Opened %s -> fd=%d\n", argv[1], dev_fd);

    test_semaphore(dev_fd);
    test_mutex(dev_fd);
    test_event(dev_fd);
    test_wait_all(dev_fd);

    close(dev_fd);

    printf("\n========== SUMMARY ==========\n");
    printf("PASS: %d  FAIL: %d\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
