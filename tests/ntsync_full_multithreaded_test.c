/*
 * ntsync_full_test.c
 *
 * Comprehensive test harness for ntsync driver using the real ntsync.h ABI.
 * Covers both single-threaded functional tests and multithreaded correctness
 * tests exercising blocking waits, cross-thread signaling, and object lifetime.
 *
 * Compile (Linux):
 *   cc -O2 -Wall -pthread -o ntsync_full_test ntsync_full_test.c
 *
 * Compile (FreeBSD):
 *   cc -O2 -Wall -lpthread -o ntsync_full_test ntsync_full_test.c
 *
 * Run:
 *   sudo ./ntsync_full_test /dev/ntsync
 *
 * Exit code: 0 if all tests pass, 1 if any fail.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <pthread.h>
#include <time.h>
#include <stdatomic.h>

#include "ntsync.h"

/* ========================================================================
 * Global test state
 * ====================================================================== */

static int g_pass = 0;
static int g_fail = 0;
static int g_dev_fd = -1;           /* control device fd, set in main */

/* Serialise CHECK output so interleaved thread prints don't corrupt lines. */
static pthread_mutex_t g_print_mtx = PTHREAD_MUTEX_INITIALIZER;

#define CHECK(cond, fmt, ...) do {                                          \
    pthread_mutex_lock(&g_print_mtx);                                       \
    if (cond) {                                                             \
        printf("  PASS: " fmt "\n", ##__VA_ARGS__);                         \
        g_pass++;                                                           \
    } else {                                                                \
        fprintf(stderr, "  FAIL [%s:%d]: " fmt "\n",                        \
                __func__, __LINE__, ##__VA_ARGS__);                         \
        g_fail++;                                                           \
    }                                                                       \
    pthread_mutex_unlock(&g_print_mtx);                                     \
} while (0)

/* ========================================================================
 * Timing helpers
 * ====================================================================== */

/* Return monotonic time in nanoseconds. */
static uint64_t now_ns(void)
{
    struct timespec ts;
#ifdef CLOCK_MONOTONIC
    clock_gettime(CLOCK_MONOTONIC, &ts);
#else
    clock_gettime(CLOCK_REALTIME, &ts);
#endif
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/*
 * Absolute monotonic timeout for ntsync_wait_args.timeout.
 * delay_ms == 0  -> pass 0 (immediate / poll)
 * delay_ms == -1 -> pass UINT64_MAX (infinite)
 */
static uint64_t deadline_ns(int delay_ms)
{
    if (delay_ms < 0)  return UINT64_MAX;
    if (delay_ms == 0) return 0;
    return now_ns() + (uint64_t)delay_ms * 1000000ULL;
}

/* Sleep for ms milliseconds. */
static void sleep_ms(int ms)
{
    struct timespec ts = {
        .tv_sec  = ms / 1000,
        .tv_nsec = (ms % 1000) * 1000000L,
    };
    nanosleep(&ts, NULL);
}

/* ========================================================================
 * ioctl wrappers
 * ====================================================================== */

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

/* Silent variant used in thread bodies where we don't want noise. */
static int do_ioctl_q(int fd, unsigned long cmd, void *arg)
{
    errno = 0;
    return ioctl(fd, cmd, arg);
}

static void safe_close(int *fd)
{
    if (*fd >= 0) { close(*fd); *fd = -1; }
}

/* ========================================================================
 * Object creation helpers
 * ====================================================================== */

static int create_sem(int count, int max)
{
    struct ntsync_sem_args a = { .count = (uint32_t)count,
                                 .max   = (uint32_t)max };
    int fd = ioctl(g_dev_fd, NTSYNC_IOC_CREATE_SEM, &a);
    if (fd < 0) perror("CREATE_SEM");
    return fd;
}

static int create_mutex(uint32_t owner, uint32_t count)
{
    struct ntsync_mutex_args a = { .owner = owner, .count = count };
    int fd = ioctl(g_dev_fd, NTSYNC_IOC_CREATE_MUTEX, &a);
    if (fd < 0) perror("CREATE_MUTEX");
    return fd;
}

static int create_event(bool manual, bool signaled)
{
    struct ntsync_event_args a = { .manual   = manual ? 1u : 0u,
                                   .signaled = signaled ? 1u : 0u };
    int fd = ioctl(g_dev_fd, NTSYNC_IOC_CREATE_EVENT, &a);
    if (fd < 0) perror("CREATE_EVENT");
    return fd;
}

/*
 * wait_any_fds: wait on an array of fds with a relative deadline.
 *
 *   fds[]     - object fd array
 *   n         - element count
 *   owner     - mutex owner hint (0 for non-mutex waits)
 *   alert_fd  - alert fd, or -1
 *   delay_ms  - 0=poll, -1=infinite, >0=relative ms timeout
 *   out_index - receives signaled index on success
 *
 * Returns 0 on success, -1 on error (errno set).
 */
static int wait_any_fds(int *fds, uint32_t n, uint32_t owner,
                        int alert_fd, int delay_ms, uint32_t *out_index)
{
    uint32_t objs[NTSYNC_MAX_WAIT_COUNT];
    for (uint32_t i = 0; i < n; i++)
        objs[i] = (uint32_t)fds[i];

    struct ntsync_wait_args wa = {
        .timeout = deadline_ns(delay_ms),
        .objs    = (uint64_t)(uintptr_t)objs,
        .count   = n,
        .index   = (uint32_t)-1,
        .owner   = owner,
        .alert   = (alert_fd >= 0) ? (uint32_t)alert_fd : 0,
    };
    int rc = do_ioctl_q(g_dev_fd, NTSYNC_IOC_WAIT_ANY, &wa);
    if (rc == 0 && out_index)
        *out_index = wa.index;
    return rc;
}

static int wait_all_fds(int *fds, uint32_t n, uint32_t owner,
                        int delay_ms)
{
    uint32_t objs[NTSYNC_MAX_WAIT_COUNT];
    for (uint32_t i = 0; i < n; i++)
        objs[i] = (uint32_t)fds[i];

    struct ntsync_wait_args wa = {
        .timeout = deadline_ns(delay_ms),
        .objs    = (uint64_t)(uintptr_t)objs,
        .count   = n,
        .index   = (uint32_t)-1,
        .owner   = owner,
    };
    return do_ioctl_q(g_dev_fd, NTSYNC_IOC_WAIT_ALL, &wa);
}

/* ========================================================================
 * Single-threaded baseline tests
 * ====================================================================== */

static int test_semaphore(void)
{
    int sem_fd = -1, rc;
    printf("\n========== SEMAPHORE TESTS (single-threaded) ==========\n");

    printf("\n-- Create semaphore (count=2 max=10) --\n");
    sem_fd = create_sem(2, 10);
    CHECK(sem_fd >= 0, "CREATE_SEM returned valid fd=%d", sem_fd);
    if (sem_fd < 0) return -1;

    printf("\n-- Read initial state --\n");
    { struct ntsync_sem_args rd = {0};
      rc = do_ioctl(sem_fd, NTSYNC_IOC_SEM_READ, &rd, "SEM_READ");
      CHECK(rc == 0,        "SEM_READ succeeded");
      CHECK(rd.count == 2,  "initial count=2 (got %u)", rd.count);
      CHECK(rd.max   == 10, "initial max=10 (got %u)",  rd.max); }

    printf("\n-- Release by 3 (2->5) --\n");
    { __u32 add = 3;
      rc = do_ioctl(sem_fd, NTSYNC_IOC_SEM_RELEASE, &add, "SEM_RELEASE(+3)");
      CHECK(rc == 0,  "SEM_RELEASE succeeded");
      CHECK(add == 2, "previous count was 2 (got %u)", add);
      struct ntsync_sem_args rd = {0};
      do_ioctl(sem_fd, NTSYNC_IOC_SEM_READ, &rd, "SEM_READ after release");
      CHECK(rd.count == 5, "count is now 5 (got %u)", rd.count); }

    printf("\n-- Overflow: release by 6 (5+6=11>max=10, expect EOVERFLOW) --\n");
    { __u32 add = 6;
      rc = do_ioctl(sem_fd, NTSYNC_IOC_SEM_RELEASE, &add, "SEM_RELEASE(overflow)");
      CHECK(rc == -1 && errno == EOVERFLOW,
            "EOVERFLOW on overflow (rc=%d errno=%d)", rc, errno); }

    printf("\n-- WAIT_ANY on signaled sem (count=5, expect immediate) --\n");
    { uint32_t idx = -1;
      int fds[1] = {sem_fd};
      rc = wait_any_fds(fds, 1, 0, -1, 0, &idx);
      CHECK(rc == 0,   "WAIT_ANY succeeded");
      CHECK(idx == 0,  "signaled index=0 (got %u)", idx);
      struct ntsync_sem_args rd = {0};
      do_ioctl(sem_fd, NTSYNC_IOC_SEM_READ, &rd, "SEM_READ after wait");
      CHECK(rd.count == 4, "count decremented to 4 (got %u)", rd.count); }

    printf("\n-- Drain to 0, then expect ETIMEDOUT --\n");
    { for (int i = 0; i < 4; i++) {
          int fds[1] = {sem_fd};
          rc = wait_any_fds(fds, 1, 0, -1, 0, NULL);
          CHECK(rc == 0, "drain step %d", i+1); }
      int fds[1] = {sem_fd};
      rc = wait_any_fds(fds, 1, 0, -1, 0, NULL);
      CHECK(rc == -1 && errno == ETIMEDOUT,
            "ETIMEDOUT on empty sem (rc=%d errno=%d)", rc, errno); }

    printf("\n-- Multi-object WAIT_ANY: two sems, only idx 1 signaled --\n");
    { int s2 = create_sem(0,5), s3 = create_sem(1,5);
      CHECK(s2 >= 0 && s3 >= 0, "helper sems created");
      if (s2 >= 0 && s3 >= 0) {
          int fds[2] = {s2, s3}; uint32_t idx = -1;
          rc = wait_any_fds(fds, 2, 0, -1, 0, &idx);
          CHECK(rc == 0,  "WAIT_ANY(2 objs) succeeded");
          CHECK(idx == 1, "signaled index=1 (got %u)", idx); }
      safe_close(&s2); safe_close(&s3); }

    safe_close(&sem_fd);
    return 0;
}

static int test_mutex(void)
{
    int mutex_fd = -1, rc;
    uint32_t owner = (uint32_t)getpid();
    printf("\n========== MUTEX TESTS (single-threaded) ==========\n");

    printf("\n-- Create mutex (owner=pid count=1) --\n");
    mutex_fd = create_mutex(owner, 1);
    CHECK(mutex_fd >= 0, "CREATE_MUTEX returned valid fd=%d", mutex_fd);
    if (mutex_fd < 0) return -1;

    printf("\n-- Read initial state --\n");
    { struct ntsync_mutex_args rd = {0};
      rc = do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd, "MUTEX_READ");
      CHECK(rc == 0,            "MUTEX_READ succeeded");
      CHECK(rd.owner == owner,  "owner=pid (got %u)", rd.owner);
      CHECK(rd.count == 1,      "count=1 (got %u)",   rd.count); }

    printf("\n-- Unlock --\n");
    { struct ntsync_mutex_args ul = { .owner = owner, .count = 0 };
      rc = do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_UNLOCK, &ul, "MUTEX_UNLOCK");
      CHECK(rc == 0, "MUTEX_UNLOCK succeeded");
      struct ntsync_mutex_args rd = {0};
      do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd, "MUTEX_READ after unlock");
      CHECK(rd.owner == 0, "owner cleared");
      CHECK(rd.count == 0, "count=0"); }

    printf("\n-- WAIT_ANY on unowned mutex --\n");
    { int fds[1] = {mutex_fd}; uint32_t idx = -1;
      rc = wait_any_fds(fds, 1, owner, -1, 0, &idx);
      CHECK(rc == 0,  "WAIT_ANY succeeded");
      CHECK(idx == 0, "signaled index=0");
      struct ntsync_mutex_args rd = {0};
      do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd, "MUTEX_READ");
      CHECK(rd.owner == owner, "mutex owned by pid");
      CHECK(rd.count == 1,     "count=1"); }

    printf("\n-- Kill mutex --\n");
    { __u32 ko = owner;
      rc = do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_KILL, &ko, "MUTEX_KILL");
      CHECK(rc == 0, "MUTEX_KILL succeeded");
      struct ntsync_mutex_args rd = {0};
      do_ioctl(mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd, "MUTEX_READ after kill");
      CHECK(rd.owner == 0, "owner cleared after kill");
      CHECK(rd.count == 0, "count=0 after kill"); }

    safe_close(&mutex_fd);
    return 0;
}

static int test_event(void)
{
    int ev_fd = -1, rc;
    printf("\n========== EVENT TESTS (single-threaded) ==========\n");

    printf("\n-- Create manual-reset event (signaled=0) --\n");
    ev_fd = create_event(true, false);
    CHECK(ev_fd >= 0, "CREATE_EVENT returned valid fd=%d", ev_fd);
    if (ev_fd < 0) return -1;

    printf("\n-- Set / reset / pulse --\n");
    { __u32 prev = 0;
      rc = do_ioctl(ev_fd, NTSYNC_IOC_EVENT_SET, &prev, "EVENT_SET");
      CHECK(rc == 0 && prev == 0, "SET: rc=%d prev=%u", rc, prev);
      prev = 0;
      rc = do_ioctl(ev_fd, NTSYNC_IOC_EVENT_RESET, &prev, "EVENT_RESET");
      CHECK(rc == 0 && prev == 1, "RESET: rc=%d prev=%u", rc, prev);
      prev = 0;
      rc = do_ioctl(ev_fd, NTSYNC_IOC_EVENT_PULSE, &prev, "EVENT_PULSE");
      CHECK(rc == 0 && prev == 0, "PULSE: rc=%d prev=%u", rc, prev);
      struct ntsync_event_args rd = {0};
      do_ioctl(ev_fd, NTSYNC_IOC_EVENT_READ, &rd, "EVENT_READ after pulse");
      CHECK(rd.signaled == 0, "unsignaled after pulse (no waiter)"); }

    printf("\n-- WAIT_ANY on unsignaled event -> ETIMEDOUT --\n");
    { int fds[1] = {ev_fd};
      rc = wait_any_fds(fds, 1, 0, -1, 0, NULL);
      CHECK(rc == -1 && errno == ETIMEDOUT,
            "ETIMEDOUT (rc=%d errno=%d)", rc, errno); }

    printf("\n-- Set then WAIT_ANY on manual-reset (stays signaled) --\n");
    { __u32 prev = 0;
      do_ioctl_q(ev_fd, NTSYNC_IOC_EVENT_SET, &prev);
      int fds[1] = {ev_fd}; uint32_t idx = -1;
      rc = wait_any_fds(fds, 1, 0, -1, 0, &idx);
      CHECK(rc == 0 && idx == 0, "WAIT_ANY succeeded");
      struct ntsync_event_args rd = {0};
      do_ioctl(ev_fd, NTSYNC_IOC_EVENT_READ, &rd, "EVENT_READ");
      CHECK(rd.signaled == 1, "manual-reset stays signaled"); }

    printf("\n-- Auto-reset event: signaled cleared after WAIT_ANY --\n");
    { int aev = create_event(false, true);
      CHECK(aev >= 0, "auto-reset event created");
      if (aev >= 0) {
          int fds[1] = {aev};
          rc = wait_any_fds(fds, 1, 0, -1, 0, NULL);
          CHECK(rc == 0, "WAIT_ANY on auto-reset succeeded");
          struct ntsync_event_args rd = {0};
          do_ioctl(aev, NTSYNC_IOC_EVENT_READ, &rd, "EVENT_READ");
          CHECK(rd.signaled == 0, "auto-reset event now unsignaled"); }
      safe_close(&aev); }

    safe_close(&ev_fd);
    return 0;
}

static int test_wait_all(void)
{
    int sem_fd = -1, ev_fd = -1, rc;
    printf("\n========== WAIT_ALL TESTS (single-threaded) ==========\n");

    printf("\n-- WAIT_ALL: two signaled objects -> success --\n");
    sem_fd = create_sem(1, 5);
    ev_fd  = create_event(true, true);
    CHECK(sem_fd >= 0 && ev_fd >= 0, "objects created");
    if (sem_fd >= 0 && ev_fd >= 0) {
        int fds[2] = {sem_fd, ev_fd};
        rc = wait_all_fds(fds, 2, 0, 0);
        CHECK(rc == 0, "WAIT_ALL succeeded (rc=%d)", rc); }

    printf("\n-- WAIT_ALL: one unsignaled -> ETIMEDOUT --\n");
    if (sem_fd >= 0 && ev_fd >= 0) {
        int fds[2] = {sem_fd, ev_fd};
        rc = wait_all_fds(fds, 2, 0, 0);
        CHECK(rc == -1 && errno == ETIMEDOUT,
              "ETIMEDOUT (rc=%d errno=%d)", rc, errno); }

    safe_close(&sem_fd);
    safe_close(&ev_fd);
    return 0;
}

/* ========================================================================
 * Multithreaded test infrastructure
 *
 * Each test uses a small shared context struct passed to worker threads.
 * Synchronisation between the main thread (verifier) and workers uses a
 * POSIX barrier so both sides are ready before the race begins, and
 * atomic_int flags to communicate results without additional locking.
 * ====================================================================== */

static void barrier_wait(pthread_barrier_t *b)
{
    int r = pthread_barrier_wait(b);
    if (r != 0 && r != PTHREAD_BARRIER_SERIAL_THREAD) {
        fprintf(stderr, "pthread_barrier_wait: %s\n", strerror(r));
        abort();
    }
}

/* ========================================================================
 * MT Test 1: Semaphore blocking wake
 *
 * Main thread blocks in WAIT_ANY on sem (count=0).
 * Worker sleeps 50ms then releases by 1.
 * Main verifies wakeup, correct index, and count consumed.
 * ====================================================================== */

struct sem_release_ctx {
    int               sem_fd;
    pthread_barrier_t barrier;
    atomic_int        release_done;
};

static void *sem_release_worker(void *arg)
{
    struct sem_release_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    sleep_ms(50);
    __u32 add = 1;
    do_ioctl_q(ctx->sem_fd, NTSYNC_IOC_SEM_RELEASE, &add);
    atomic_store(&ctx->release_done, 1);
    return NULL;
}

static void test_mt_sem_blocking_wake(void)
{
    printf("\n-- MT: semaphore blocking wake --\n");

    struct sem_release_ctx ctx;
    ctx.sem_fd = create_sem(0, 5);
    atomic_init(&ctx.release_done, 0);
    pthread_barrier_init(&ctx.barrier, NULL, 2);

    pthread_t tid;
    pthread_create(&tid, NULL, sem_release_worker, &ctx);
    barrier_wait(&ctx.barrier);

    uint64_t t0 = now_ns();
    int fds[1] = {ctx.sem_fd};
    uint32_t idx = (uint32_t)-1;
    int rc = wait_any_fds(fds, 1, 0, -1, -1 /*infinite*/, &idx);
    uint64_t elapsed_ms = (now_ns() - t0) / 1000000;

    CHECK(rc == 0,  "MT sem wake: WAIT_ANY returned 0");
    CHECK(idx == 0, "MT sem wake: signaled index=0 (got %u)", idx);
    CHECK(atomic_load(&ctx.release_done),
          "MT sem wake: worker completed release before wakeup");
    CHECK(elapsed_ms >= 30 && elapsed_ms < 2000,
          "MT sem wake: elapsed ~50ms (got %"PRIu64"ms)", elapsed_ms);

    struct ntsync_sem_args rd = {0};
    do_ioctl_q(ctx.sem_fd, NTSYNC_IOC_SEM_READ, &rd);
    CHECK(rd.count == 0, "MT sem wake: count back to 0 (got %u)", rd.count);

    pthread_join(tid, NULL);
    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&ctx.sem_fd);
}

/* ========================================================================
 * MT Test 2: Multiple threads racing on a semaphore
 *
 * N_SEM_WAITERS threads all block on sem (count=0).
 * Main releases N_SEM_WAITERS tokens.  Each waiter wakes exactly once.
 * ====================================================================== */

#define N_SEM_WAITERS 4

struct sem_race_ctx {
    int               sem_fd;
    pthread_barrier_t barrier;
    atomic_int        wake_count;
};

static void *sem_race_waiter(void *arg)
{
    struct sem_race_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    int fds[1] = {ctx->sem_fd};
    int rc = wait_any_fds(fds, 1, 0, -1, 5000, NULL);
    if (rc == 0) atomic_fetch_add(&ctx->wake_count, 1);
    return NULL;
}

static void test_mt_sem_multi_waiter(void)
{
    printf("\n-- MT: %d threads race on semaphore --\n", N_SEM_WAITERS);

    struct sem_race_ctx ctx;
    ctx.sem_fd = create_sem(0, N_SEM_WAITERS + 1);
    atomic_init(&ctx.wake_count, 0);
    pthread_barrier_init(&ctx.barrier, NULL, N_SEM_WAITERS + 1);

    pthread_t tids[N_SEM_WAITERS];
    for (int i = 0; i < N_SEM_WAITERS; i++)
        pthread_create(&tids[i], NULL, sem_race_waiter, &ctx);

    barrier_wait(&ctx.barrier);
    sleep_ms(50);

    __u32 add = N_SEM_WAITERS;
    do_ioctl_q(ctx.sem_fd, NTSYNC_IOC_SEM_RELEASE, &add);

    for (int i = 0; i < N_SEM_WAITERS; i++)
        pthread_join(tids[i], NULL);

    int woken = atomic_load(&ctx.wake_count);
    CHECK(woken == N_SEM_WAITERS,
          "MT sem race: all %d waiters woken (got %d)", N_SEM_WAITERS, woken);

    struct ntsync_sem_args rd = {0};
    do_ioctl_q(ctx.sem_fd, NTSYNC_IOC_SEM_READ, &rd);
    CHECK(rd.count == 0, "MT sem race: count drained to 0 (got %u)", rd.count);

    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&ctx.sem_fd);
}

/* ========================================================================
 * MT Test 3: Manual-reset event wakes all waiters simultaneously
 *
 * N_EV_WAITERS threads block on manual-reset event.  Main calls
 * EVENT_SET.  All N waiters must wake; the event stays set.
 * ====================================================================== */

#define N_EV_WAITERS 4

struct event_broadcast_ctx {
    int               ev_fd;
    pthread_barrier_t barrier;
    atomic_int        wake_count;
};

static void *event_broadcast_waiter(void *arg)
{
    struct event_broadcast_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    int fds[1] = {ctx->ev_fd};
    int rc = wait_any_fds(fds, 1, 0, -1, 5000, NULL);
    if (rc == 0) atomic_fetch_add(&ctx->wake_count, 1);
    return NULL;
}

static void test_mt_event_broadcast(void)
{
    printf("\n-- MT: manual-reset event wakes all %d waiters --\n", N_EV_WAITERS);

    struct event_broadcast_ctx ctx;
    ctx.ev_fd = create_event(true /*manual*/, false);
    atomic_init(&ctx.wake_count, 0);
    pthread_barrier_init(&ctx.barrier, NULL, N_EV_WAITERS + 1);

    pthread_t tids[N_EV_WAITERS];
    for (int i = 0; i < N_EV_WAITERS; i++)
        pthread_create(&tids[i], NULL, event_broadcast_waiter, &ctx);

    barrier_wait(&ctx.barrier);
    sleep_ms(50);

    __u32 prev = 0;
    do_ioctl_q(ctx.ev_fd, NTSYNC_IOC_EVENT_SET, &prev);

    for (int i = 0; i < N_EV_WAITERS; i++)
        pthread_join(tids[i], NULL);

    int woken = atomic_load(&ctx.wake_count);
    CHECK(woken == N_EV_WAITERS,
          "MT event broadcast: all %d waiters woken (got %d)",
          N_EV_WAITERS, woken);

    struct ntsync_event_args rd = {0};
    do_ioctl_q(ctx.ev_fd, NTSYNC_IOC_EVENT_READ, &rd);
    CHECK(rd.signaled == 1,
          "MT event broadcast: manual-reset event still signaled");

    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&ctx.ev_fd);
}

/* ========================================================================
 * MT Test 4: Auto-reset event releases exactly one waiter per SET
 *
 * N_EV_WAITERS threads block on auto-reset event.
 * Main calls EVENT_SET N times (with pauses).
 * Each set releases exactly one waiter and the event immediately clears.
 * ====================================================================== */

struct event_autoreset_ctx {
    int               ev_fd;
    pthread_barrier_t barrier;
    atomic_int        wake_count;
};

static void *event_autoreset_waiter(void *arg)
{
    struct event_autoreset_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    int fds[1] = {ctx->ev_fd};
    int rc = wait_any_fds(fds, 1, 0, -1, 5000, NULL);
    if (rc == 0) atomic_fetch_add(&ctx->wake_count, 1);
    return NULL;
}

static void test_mt_event_autoreset(void)
{
    printf("\n-- MT: auto-reset event releases one waiter per SET --\n");

    struct event_autoreset_ctx ctx;
    ctx.ev_fd = create_event(false /*auto*/, false);
    atomic_init(&ctx.wake_count, 0);
    pthread_barrier_init(&ctx.barrier, NULL, N_EV_WAITERS + 1);

    pthread_t tids[N_EV_WAITERS];
    for (int i = 0; i < N_EV_WAITERS; i++)
        pthread_create(&tids[i], NULL, event_autoreset_waiter, &ctx);

    barrier_wait(&ctx.barrier);
    sleep_ms(50);

    for (int i = 0; i < N_EV_WAITERS; i++) {
        __u32 prev = 0;
        do_ioctl_q(ctx.ev_fd, NTSYNC_IOC_EVENT_SET, &prev);
        sleep_ms(20);   /* yield so released waiter can run and auto-reset */
    }

    for (int i = 0; i < N_EV_WAITERS; i++)
        pthread_join(tids[i], NULL);

    int woken = atomic_load(&ctx.wake_count);
    CHECK(woken == N_EV_WAITERS,
          "MT auto-reset: all %d waiters woken (got %d)", N_EV_WAITERS, woken);

    struct ntsync_event_args rd = {0};
    do_ioctl_q(ctx.ev_fd, NTSYNC_IOC_EVENT_READ, &rd);
    CHECK(rd.signaled == 0,
          "MT auto-reset: event unsignaled after all waiters consumed");

    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&ctx.ev_fd);
}

/* ========================================================================
 * MT Test 5: WAIT_ALL unblocked when last needed object becomes signaled
 *
 * Worker blocks in WAIT_ALL on {sem(count=0), event(signaled)}.
 * Main releases sem by 1 after 50ms.
 * Worker should wake; both objects must be consumed.
 * ====================================================================== */

struct wait_all_ctx {
    int               sem_fd;
    int               ev_fd;
    pthread_barrier_t barrier;
    atomic_int        wait_rc;
};

static void *wait_all_worker(void *arg)
{
    struct wait_all_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    int fds[2] = {ctx->sem_fd, ctx->ev_fd};
    int rc = wait_all_fds(fds, 2, 0, 5000);
    atomic_store(&ctx->wait_rc, rc == 0 ? 0 : errno);
    return NULL;
}

static void test_mt_wait_all_blocking(void)
{
    printf("\n-- MT: WAIT_ALL unblocked by late semaphore release --\n");

    struct wait_all_ctx ctx;
    ctx.sem_fd = create_sem(0, 5);
    ctx.ev_fd  = create_event(false /*auto-reset*/, true);
    atomic_init(&ctx.wait_rc, -999);
    pthread_barrier_init(&ctx.barrier, NULL, 2);

    pthread_t tid;
    pthread_create(&tid, NULL, wait_all_worker, &ctx);
    barrier_wait(&ctx.barrier);
    sleep_ms(50);

    uint64_t t0 = now_ns();
    __u32 add = 1;
    do_ioctl_q(ctx.sem_fd, NTSYNC_IOC_SEM_RELEASE, &add);

    pthread_join(tid, NULL);
    uint64_t elapsed_ms = (now_ns() - t0) / 1000000;

    CHECK(atomic_load(&ctx.wait_rc) == 0,
          "MT WAIT_ALL: worker returned 0 (got errno=%d)",
          atomic_load(&ctx.wait_rc));
    CHECK(elapsed_ms < 2000,
          "MT WAIT_ALL: woke within 2s (got %"PRIu64"ms)", elapsed_ms);

    struct ntsync_sem_args sr = {0};
    do_ioctl_q(ctx.sem_fd, NTSYNC_IOC_SEM_READ, &sr);
    CHECK(sr.count == 0,
          "MT WAIT_ALL: sem count consumed (got %u)", sr.count);

    struct ntsync_event_args er = {0};
    do_ioctl_q(ctx.ev_fd, NTSYNC_IOC_EVENT_READ, &er);
    CHECK(er.signaled == 0,
          "MT WAIT_ALL: auto-reset event cleared (got %u)", er.signaled);

    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&ctx.sem_fd);
    safe_close(&ctx.ev_fd);
}

/* ========================================================================
 * MT Test 6: Mutex cross-thread handoff
 *
 * Main holds mutex (owner=main_owner).  Worker blocks in WAIT_ANY.
 * Main unlocks; worker should acquire and own the mutex.
 * ====================================================================== */

struct mutex_handoff_ctx {
    int               mutex_fd;
    uint32_t          worker_owner;
    pthread_barrier_t barrier;
    atomic_int        wait_rc;
    atomic_int        got_owner;
};

static void *mutex_handoff_worker(void *arg)
{
    struct mutex_handoff_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    int fds[1] = {ctx->mutex_fd};
    int rc = wait_any_fds(fds, 1, ctx->worker_owner, -1, 5000, NULL);
    atomic_store(&ctx->wait_rc, rc == 0 ? 0 : errno);
    if (rc == 0) {
        struct ntsync_mutex_args rd = {0};
        do_ioctl_q(ctx->mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd);
        atomic_store(&ctx->got_owner, (int)rd.owner);
    }
    return NULL;
}

static void test_mt_mutex_handoff(void)
{
    printf("\n-- MT: mutex handoff between threads --\n");

    uint32_t main_owner   = (uint32_t)getpid();
    uint32_t worker_owner = main_owner + 1;

    struct mutex_handoff_ctx ctx;
    ctx.mutex_fd     = create_mutex(main_owner, 1);
    ctx.worker_owner = worker_owner;
    atomic_init(&ctx.wait_rc,   -999);
    atomic_init(&ctx.got_owner,  0);
    pthread_barrier_init(&ctx.barrier, NULL, 2);

    pthread_t tid;
    pthread_create(&tid, NULL, mutex_handoff_worker, &ctx);
    barrier_wait(&ctx.barrier);
    sleep_ms(50);

    struct ntsync_mutex_args ul = { .owner = main_owner, .count = 0 };
    do_ioctl_q(ctx.mutex_fd, NTSYNC_IOC_MUTEX_UNLOCK, &ul);

    pthread_join(tid, NULL);

    CHECK(atomic_load(&ctx.wait_rc) == 0,
          "MT mutex handoff: worker WAIT_ANY returned 0 (got errno=%d)",
          atomic_load(&ctx.wait_rc));
    CHECK((uint32_t)atomic_load(&ctx.got_owner) == worker_owner,
          "MT mutex handoff: worker owns mutex (owner=%u, want %u)",
          (uint32_t)atomic_load(&ctx.got_owner), worker_owner);

    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&ctx.mutex_fd);
}

/* ========================================================================
 * MT Test 7: MUTEX_KILL delivers EOWNERDEAD to waiting thread
 *
 * Worker blocks on mutex owned by main.  Main kills mutex.
 * Worker must wake with EOWNERDEAD.
 * ====================================================================== */

struct mutex_kill_ctx {
    int               mutex_fd;
    uint32_t          worker_owner;
    pthread_barrier_t barrier;
    atomic_int        wait_errno;
};

static void *mutex_kill_waiter(void *arg)
{
    struct mutex_kill_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    int fds[1] = {ctx->mutex_fd};
    int rc = wait_any_fds(fds, 1, ctx->worker_owner, -1, 5000, NULL);
    atomic_store(&ctx->wait_errno, rc == 0 ? 0 : errno);
    return NULL;
}

static void test_mt_mutex_kill_eownerdead(void)
{
    printf("\n-- MT: MUTEX_KILL delivers EOWNERDEAD to waiter --\n");

    uint32_t main_owner   = (uint32_t)getpid();
    uint32_t worker_owner = main_owner + 1;

    struct mutex_kill_ctx ctx;
    ctx.mutex_fd     = create_mutex(main_owner, 1);
    ctx.worker_owner = worker_owner;
    atomic_init(&ctx.wait_errno, -999);
    pthread_barrier_init(&ctx.barrier, NULL, 2);

    pthread_t tid;
    pthread_create(&tid, NULL, mutex_kill_waiter, &ctx);
    barrier_wait(&ctx.barrier);
    sleep_ms(50);

    __u32 ko = main_owner;
    do_ioctl_q(ctx.mutex_fd, NTSYNC_IOC_MUTEX_KILL, &ko);

    pthread_join(tid, NULL);

    int we = atomic_load(&ctx.wait_errno);
    CHECK(we == EOWNERDEAD,
          "MT mutex kill: worker got EOWNERDEAD (got errno=%d)", we);

    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&ctx.mutex_fd);
}

/* ========================================================================
 * MT Test 8: Alert event interrupts a blocking WAIT_ANY
 *
 * Worker blocks in WAIT_ANY(sem=unsignaled, alert=alert_ev).
 * Main sets the alert event.  Worker should wake at the alert index (=1,
 * since there is 1 normal object before the alert slot).
 * ====================================================================== */

struct alert_ctx {
    int               sem_fd;
    int               alert_fd;
    pthread_barrier_t barrier;
    atomic_int        wait_rc;
    atomic_uint       signaled_idx;
};

static void *alert_waiter(void *arg)
{
    struct alert_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);

    uint32_t objs[1] = { (uint32_t)ctx->sem_fd };
    struct ntsync_wait_args wa = {
        .timeout = deadline_ns(5000),
        .objs    = (uint64_t)(uintptr_t)objs,
        .count   = 1,
        .index   = (uint32_t)-1,
        .alert   = (uint32_t)ctx->alert_fd,
    };
    int rc = do_ioctl_q(g_dev_fd, NTSYNC_IOC_WAIT_ANY, &wa);
    atomic_store(&ctx->wait_rc,      rc == 0 ? 0 : errno);
    atomic_store(&ctx->signaled_idx, wa.index);
    return NULL;
}

static void test_mt_alert_event(void)
{
    printf("\n-- MT: alert event interrupts WAIT_ANY --\n");

    struct alert_ctx ctx;
    ctx.sem_fd   = create_sem(0, 5);
    ctx.alert_fd = create_event(false /*auto-reset*/, false);
    atomic_init(&ctx.wait_rc,      -999);
    atomic_init(&ctx.signaled_idx, (uint32_t)-1);
    pthread_barrier_init(&ctx.barrier, NULL, 2);

    pthread_t tid;
    pthread_create(&tid, NULL, alert_waiter, &ctx);
    barrier_wait(&ctx.barrier);
    sleep_ms(50);

    __u32 prev = 0;
    do_ioctl_q(ctx.alert_fd, NTSYNC_IOC_EVENT_SET, &prev);

    pthread_join(tid, NULL);

    CHECK(atomic_load(&ctx.wait_rc) == 0,
          "MT alert: WAIT_ANY returned 0 (got errno=%d)",
          atomic_load(&ctx.wait_rc));
    /*
     * Alert object is appended after the normal objects internally.
     * With count=1 normal object the alert lands at index 1.
     */
    CHECK(atomic_load(&ctx.signaled_idx) == 1,
          "MT alert: signaled index=1 (alert slot) (got %u)",
          atomic_load(&ctx.signaled_idx));

    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&ctx.sem_fd);
    safe_close(&ctx.alert_fd);
}

/* ========================================================================
 * MT Test 9: Timeout accuracy
 *
 * Block on unsignaled sem for 100ms.  Verify ETIMEDOUT and elapsed time
 * is in [80ms, 500ms].
 * ====================================================================== */

static void test_mt_timeout_accuracy(void)
{
    printf("\n-- MT: timeout accuracy (100ms wait on unsignaled sem) --\n");

    int sem_fd = create_sem(0, 1);

    uint64_t t0 = now_ns();
    int fds[1] = {sem_fd};
    int rc = wait_any_fds(fds, 1, 0, -1, 100 /*ms*/, NULL);
    uint64_t elapsed_ms = (now_ns() - t0) / 1000000;

    CHECK(rc == -1 && errno == ETIMEDOUT,
          "timeout: ETIMEDOUT returned (rc=%d errno=%d)", rc, errno);
    CHECK(elapsed_ms >= 80 && elapsed_ms < 500,
          "timeout: elapsed %"PRIu64"ms (want 80-500ms)", elapsed_ms);

    safe_close(&sem_fd);
}

/* ========================================================================
 * MT Test 10: Close object while thread is blocked (deletion-abort)
 *
 * Exercises FreeBSD's deleted/active_waiters drain.
 *
 * Worker blocks in WAIT_ANY on sem (count=0, 10s timeout).
 * Main closes its copy of the fd after 50ms.
 *
 * FreeBSD: WAIT_ANY returns ETIMEDOUT (aborted path) and close() does
 *          not return until the waiter has unregistered.
 * Linux:   fget holds a reference so close() does not disturb the wait;
 *          worker times out naturally after 10s.
 *
 * We accept both outcomes; the invariant is no deadlock or crash.
 * ====================================================================== */

struct close_while_waiting_ctx {
    int               worker_fd;
    pthread_barrier_t barrier;
    atomic_int        wait_result;
};

static void *close_while_waiting_worker(void *arg)
{
    struct close_while_waiting_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    int fds[1] = {ctx->worker_fd};
    int rc = wait_any_fds(fds, 1, 0, -1, 10000, NULL);
    atomic_store(&ctx->wait_result, rc == 0 ? 0 : errno);
    return NULL;
}

static void test_mt_close_while_waiting(void)
{
    printf("\n-- MT: close object while thread is blocked in WAIT_ANY --\n");

    int sem_fd    = create_sem(0, 1);
    int worker_fd = dup(sem_fd);
    CHECK(worker_fd >= 0, "dup succeeded (worker_fd=%d)", worker_fd);

    struct close_while_waiting_ctx ctx;
    ctx.worker_fd = worker_fd;
    atomic_init(&ctx.wait_result, -999);
    pthread_barrier_init(&ctx.barrier, NULL, 2);

    pthread_t tid;
    pthread_create(&tid, NULL, close_while_waiting_worker, &ctx);
    barrier_wait(&ctx.barrier);
    sleep_ms(50);

    safe_close(&sem_fd);        /* triggers FreeBSD deletion-abort path */

    pthread_join(tid, NULL);    /* must not deadlock */

    int result = atomic_load(&ctx.wait_result);
    CHECK(result == ETIMEDOUT || result == 0 ||
          result == EBADF     || result == EINVAL,
          "MT close-while-waiting: worker exited cleanly (result=%d)", result);

    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&worker_fd);
}

/* ========================================================================
 * MT Test 11: Concurrent producer/consumer storm
 *
 * N_STORM_PRODUCERS threads each release 1 token per iter.
 * N_STORM_CONSUMERS threads each consume 1 token per iter.
 * Invariant: total_consumed + remaining_count == total_produced.
 * ====================================================================== */

#define N_STORM_PRODUCERS 4
#define N_STORM_CONSUMERS 4
#define N_STORM_ITERS     200

struct storm_ctx {
    int               sem_fd;
    pthread_barrier_t barrier;
    atomic_int        total_consumed;
    atomic_int        total_produced;
};

static void *storm_producer(void *arg)
{
    struct storm_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    for (int i = 0; i < N_STORM_ITERS; i++) {
        __u32 add = 1;
        int rc = do_ioctl_q(ctx->sem_fd, NTSYNC_IOC_SEM_RELEASE, &add);
        if (rc == 0)
            atomic_fetch_add(&ctx->total_produced, 1);
        else if (errno != EOVERFLOW)
            fprintf(stderr, "storm_producer: unexpected errno=%d\n", errno);
    }
    return NULL;
}

static void *storm_consumer(void *arg)
{
    struct storm_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    for (int i = 0; i < N_STORM_ITERS; i++) {
        int fds[1] = {ctx->sem_fd};
        int rc = wait_any_fds(fds, 1, 0, -1, 2000, NULL);
        if (rc == 0) atomic_fetch_add(&ctx->total_consumed, 1);
    }
    return NULL;
}

static void test_mt_sem_storm(void)
{
    printf("\n-- MT: concurrent producer/consumer storm (%d+%d threads, %d iters) --\n",
           N_STORM_PRODUCERS, N_STORM_CONSUMERS, N_STORM_ITERS);

    struct storm_ctx ctx;
    ctx.sem_fd = create_sem(0, N_STORM_PRODUCERS * N_STORM_ITERS);
    atomic_init(&ctx.total_consumed, 0);
    atomic_init(&ctx.total_produced, 0);
    pthread_barrier_init(&ctx.barrier, NULL,
                         N_STORM_PRODUCERS + N_STORM_CONSUMERS);

    pthread_t ptids[N_STORM_PRODUCERS], ctids[N_STORM_CONSUMERS];
    for (int i = 0; i < N_STORM_PRODUCERS; i++)
        pthread_create(&ptids[i], NULL, storm_producer, &ctx);
    for (int i = 0; i < N_STORM_CONSUMERS; i++)
        pthread_create(&ctids[i], NULL, storm_consumer, &ctx);

    for (int i = 0; i < N_STORM_PRODUCERS; i++) pthread_join(ptids[i], NULL);
    for (int i = 0; i < N_STORM_CONSUMERS; i++) pthread_join(ctids[i], NULL);

    int produced  = atomic_load(&ctx.total_produced);
    int consumed  = atomic_load(&ctx.total_consumed);
    struct ntsync_sem_args rd = {0};
    do_ioctl_q(ctx.sem_fd, NTSYNC_IOC_SEM_READ, &rd);
    int remaining = (int)rd.count;

    CHECK(consumed + remaining == produced,
          "MT storm: produced=%d consumed=%d remaining=%d (sum=%d)",
          produced, consumed, remaining, consumed + remaining);
    CHECK(consumed > 0,
          "MT storm: at least one token consumed (got %d)", consumed);

    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&ctx.sem_fd);
}

/* ========================================================================
 * MT Test 12: WAIT_ALL atomicity under concurrent modification
 *
 * Waiter:   WAIT_ALL on {sem, event} — 5s timeout.
 * Toggler:  toggles the event every ~10ms.
 * Releaser: releases 1 sem token every ~7ms.
 *
 * Eventually both are signaled simultaneously and the waiter wakes exactly
 * once, consuming both.
 * ====================================================================== */

struct wait_all_atomic_ctx {
    int               sem_fd;
    int               ev_fd;
    pthread_barrier_t barrier;
    atomic_int        waiter_rc;
    atomic_int        stop;
};

static void *wait_all_waiter(void *arg)
{
    struct wait_all_atomic_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    int fds[2] = {ctx->sem_fd, ctx->ev_fd};
    int rc = wait_all_fds(fds, 2, 0, 5000);
    atomic_store(&ctx->waiter_rc, rc == 0 ? 0 : errno);
    atomic_store(&ctx->stop, 1);
    return NULL;
}

static void *event_toggler(void *arg)
{
    struct wait_all_atomic_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    while (!atomic_load(&ctx->stop)) {
        __u32 prev = 0;
        do_ioctl_q(ctx->ev_fd, NTSYNC_IOC_EVENT_SET,   &prev);
        sleep_ms(5);
        do_ioctl_q(ctx->ev_fd, NTSYNC_IOC_EVENT_RESET, &prev);
        sleep_ms(5);
    }
    return NULL;
}

static void *sem_releaser(void *arg)
{
    struct wait_all_atomic_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    while (!atomic_load(&ctx->stop)) {
        __u32 add = 1;
        do_ioctl_q(ctx->sem_fd, NTSYNC_IOC_SEM_RELEASE, &add);
        sleep_ms(7);
    }
    return NULL;
}

static void test_mt_wait_all_atomicity(void)
{
    printf("\n-- MT: WAIT_ALL atomicity under concurrent modification --\n");

    struct wait_all_atomic_ctx ctx;
    ctx.sem_fd = create_sem(0, 100);
    ctx.ev_fd  = create_event(false /*auto-reset*/, false);
    atomic_init(&ctx.waiter_rc, -999);
    atomic_init(&ctx.stop, 0);
    pthread_barrier_init(&ctx.barrier, NULL, 3);

    pthread_t waiter_tid, toggler_tid, releaser_tid;
    pthread_create(&waiter_tid,   NULL, wait_all_waiter,  &ctx);
    pthread_create(&toggler_tid,  NULL, event_toggler,    &ctx);
    pthread_create(&releaser_tid, NULL, sem_releaser,     &ctx);

    pthread_join(waiter_tid, NULL);
    atomic_store(&ctx.stop, 1);
    pthread_join(toggler_tid,  NULL);
    pthread_join(releaser_tid, NULL);

    CHECK(atomic_load(&ctx.waiter_rc) == 0,
          "MT WAIT_ALL atomicity: waiter returned 0 (got errno=%d)",
          atomic_load(&ctx.waiter_rc));

    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&ctx.sem_fd);
    safe_close(&ctx.ev_fd);
}

/* ========================================================================
 * MT Test 13: Recursive mutex (same owner, count > 1)
 *
 * Main acquires mutex twice (recursive, count=2).
 * Thread B blocks in WAIT_ANY.
 * Main unlocks twice; Thread B acquires on the second unlock.
 * ====================================================================== */

struct recursive_mutex_ctx {
    int               mutex_fd;
    uint32_t          owner_a;
    uint32_t          owner_b;
    pthread_barrier_t barrier;
    atomic_int        b_wait_rc;
    atomic_int        b_got_owner;
};

static void *recursive_mutex_b(void *arg)
{
    struct recursive_mutex_ctx *ctx = arg;
    barrier_wait(&ctx->barrier);
    int fds[1] = {ctx->mutex_fd};
    int rc = wait_any_fds(fds, 1, ctx->owner_b, -1, 5000, NULL);
    atomic_store(&ctx->b_wait_rc, rc == 0 ? 0 : errno);
    if (rc == 0) {
        struct ntsync_mutex_args rd = {0};
        do_ioctl_q(ctx->mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd);
        atomic_store(&ctx->b_got_owner, (int)rd.owner);
    }
    return NULL;
}

static void test_mt_recursive_mutex(void)
{
    printf("\n-- MT: recursive mutex (same owner acquires twice) --\n");

    uint32_t owner_a = (uint32_t)getpid();
    uint32_t owner_b = owner_a + 2;

    struct recursive_mutex_ctx ctx;
    ctx.mutex_fd = create_mutex(0, 0);
    ctx.owner_a  = owner_a;
    ctx.owner_b  = owner_b;
    atomic_init(&ctx.b_wait_rc,   -999);
    atomic_init(&ctx.b_got_owner,  0);
    pthread_barrier_init(&ctx.barrier, NULL, 2);

    /* First acquire */
    { int fds[1] = {ctx.mutex_fd};
      int rc = wait_any_fds(fds, 1, owner_a, -1, 0, NULL);
      CHECK(rc == 0, "recursive: first acquire succeeded"); }

    /* Second acquire (recursive) */
    { int fds[1] = {ctx.mutex_fd};
      int rc = wait_any_fds(fds, 1, owner_a, -1, 0, NULL);
      CHECK(rc == 0, "recursive: second acquire succeeded");
      struct ntsync_mutex_args rd = {0};
      do_ioctl_q(ctx.mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd);
      CHECK(rd.count == 2,
            "recursive: count=2 after double acquire (got %u)", rd.count); }

    pthread_t tid;
    pthread_create(&tid, NULL, recursive_mutex_b, &ctx);
    barrier_wait(&ctx.barrier);
    sleep_ms(50);

    /* First unlock: count 2->1, still owned by A */
    { struct ntsync_mutex_args ul = { .owner = owner_a, .count = 0 };
      int rc = do_ioctl_q(ctx.mutex_fd, NTSYNC_IOC_MUTEX_UNLOCK, &ul);
      CHECK(rc == 0, "recursive: first unlock succeeded");
      struct ntsync_mutex_args rd = {0};
      do_ioctl_q(ctx.mutex_fd, NTSYNC_IOC_MUTEX_READ, &rd);
      CHECK(rd.owner == owner_a && rd.count == 1,
            "recursive: still owned by A, count=1 (owner=%u count=%u)",
            rd.owner, rd.count); }

    /* Second unlock: count 1->0, releases to B */
    { struct ntsync_mutex_args ul = { .owner = owner_a, .count = 0 };
      int rc = do_ioctl_q(ctx.mutex_fd, NTSYNC_IOC_MUTEX_UNLOCK, &ul);
      CHECK(rc == 0, "recursive: second unlock succeeded"); }

    pthread_join(tid, NULL);

    CHECK(atomic_load(&ctx.b_wait_rc) == 0,
          "recursive: Thread B acquired after A released (rc=%d)",
          atomic_load(&ctx.b_wait_rc));
    CHECK((uint32_t)atomic_load(&ctx.b_got_owner) == owner_b,
          "recursive: mutex owned by B (owner=%u want %u)",
          (uint32_t)atomic_load(&ctx.b_got_owner), owner_b);

    pthread_barrier_destroy(&ctx.barrier);
    safe_close(&ctx.mutex_fd);
}

/* ========================================================================
 * Main
 * ====================================================================== */

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <device-path>\n", argv[0]);
        return 2;
    }

    g_dev_fd = open(argv[1], O_RDWR);
    if (g_dev_fd < 0) { perror("open"); return 1; }
    printf("Opened %s -> fd=%d\n", argv[1], g_dev_fd);

    /* ---- Single-threaded baseline ---- */
    test_semaphore();
    test_mutex();
    test_event();
    test_wait_all();

    /* ---- Multithreaded ---- */
    printf("\n\n========== MULTITHREADED TESTS ==========\n");
    test_mt_sem_blocking_wake();        /* 1: blocking wake via release */
    test_mt_sem_multi_waiter();         /* 2: N threads race on one sem */
    test_mt_event_broadcast();          /* 3: manual-reset wakes all */
    test_mt_event_autoreset();          /* 4: auto-reset wakes one per SET */
    test_mt_wait_all_blocking();        /* 5: WAIT_ALL unblocked late */
    test_mt_mutex_handoff();            /* 6: mutex handoff A->B */
    test_mt_mutex_kill_eownerdead();    /* 7: MUTEX_KILL -> EOWNERDEAD */
    test_mt_alert_event();              /* 8: alert interrupts WAIT_ANY */
    test_mt_timeout_accuracy();         /* 9: timeout fires at ~100ms */
    test_mt_close_while_waiting();      /* 10: close fd while blocked */
    test_mt_sem_storm();                /* 11: producer/consumer storm */
    test_mt_wait_all_atomicity();       /* 12: WAIT_ALL atomicity */
    test_mt_recursive_mutex();          /* 13: recursive mutex count */

    close(g_dev_fd);

    printf("\n========== SUMMARY ==========\n");
    printf("PASS: %d  FAIL: %d\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
