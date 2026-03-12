// SPDX-License-Identifier: GPL-2.0-only
/*
 * ntsync.c - Kernel driver for NT synchronization primitives
 *
 * Copyright (C) 2024 Elizabeth Figura <zfigura@codeweavers.com>
 * Copyright (C) 2026 Vincent H. <tmp386@live.com> (FreeBSD version)
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/selinfo.h>
#include <sys/queue.h>
#include <sys/proc.h>
#include <sys/file.h>
#include <sys/filedesc.h>
#include <sys/fcntl.h>
#include <sys/poll.h>
#include <sys/errno.h>
#include <sys/uio.h>
#include <sys/lock.h>
#include <sys/event.h>
#include <sys/module.h>
#include <sys/time.h>
#include <sys/capsicum.h>
#ifdef COMPAT_FREEBSD32
#include <compat/freebsd32/freebsd32_util.h>
#endif
#include <sys/limits.h>
#include <sys/ioccom.h>
#include <sys/libkern.h>
#include <sys/stat.h>
#include <sys/user.h>

#include <sys/counter.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/uma.h>

#include <machine/atomic.h>

#include "ntsync.h"

/* Define NTSYNC_DEBUG at compile time (e.g. CFLAGS+="-DNTSYNC_DEBUG" in Makefile)
 * to enable verbose diagnostic output. Do NOT enable in production — generates
 * O(calls/sec) console spam that saturates dmesg and degrades performance.
 */
#ifdef NTSYNC_DEBUG
#define NTSYNC_DPRINTF(...) printf(__VA_ARGS__)
#else
#define NTSYNC_DPRINTF(...) ((void)0)
#endif

/*
 * WOW64 / COMPAT_FREEBSD32 support
 * ---------------------------------------------------------------------------
 *
 * WOW64 ("Windows 32-bit on Windows 64-bit") in Wine means running 32-bit
 * Windows applications on a 64-bit FreeBSD host.  Two distinct cases arise:
 *
 *   A. Modern Wine WOW64 mode (Wine >= 7.0, --enable-archs=i386,x86_64):
 *      A single 64-bit host process runs all NT sync operations through its
 *      64-bit ntdll.so.  All ntsync ioctls originate from a 64-bit process.
 *      No driver changes are needed; this case works without any compat layer.
 *
 *   B. Legacy 32-bit Wine (a pure i386 ELF binary on amd64 FreeBSD):
 *      The Wine process itself is 32-bit.  Its ntsync ioctls go through
 *      freebsd32_ioctl(2) -> kern_ioctl -> d_ioctl / fo_ioctl.
 *      This is the case that requires the adaptations below.
 *
 * WHY NO STRUCT TRANSLATION IS REQUIRED
 * ---------------------------------------------------------------------------
 *
 * Every ntsync ioctl argument struct is composed exclusively of __u32 and
 * __u64 fixed-width integer types.  sizeof() is therefore identical in both
 * the ILP32 (32-bit) and LP64 (64-bit) ABIs:
 *
 *   ntsync_sem_args   :  2 x __u32                        =  8 bytes
 *   ntsync_mutex_args :  2 x __u32                        =  8 bytes
 *   ntsync_event_args :  2 x __u32                        =  8 bytes
 *   ntsync_wait_args  :  2 x __u64  +  6 x __u32         = 40 bytes
 *
 * Because the ioctl command numbers encode struct size via the _IOW/_IOWR
 * macros, 32-bit Wine and 64-bit Wine produce bit-for-bit identical command
 * codes.  freebsd32_ioctl(2) zero-extends the 32-bit data pointer and calls
 * kern_ioctl -> our d_ioctl with a kernel-side copy of the correct size.
 * No translation is needed and no separate compat ioctl handler is required.
 *
 * The _Static_assert()s below fire at build time if any struct field is
 * ever changed to a pointer-sized type or if padding is inadvertently
 * introduced.  They are unconditional (not gated on COMPAT_FREEBSD32)
 * because the struct sizes must be ABI-stable on every build, not only
 * compat ones.
 *
 * WHAT DOES NEED EXPLICIT HANDLING
 * ---------------------------------------------------------------------------
 *
 *   1. ntsync_wait_args::objs  (__u64, carries a user-space pointer)
 *
 *      A 64-bit caller stores a full 64-bit user address here.  A 32-bit
 *      (WOW64) caller stores a 32-bit pointer in the low half; the upper
 *      32 bits should be zero but cannot be trusted from an adversarial
 *      process.  ntsync_objs_uptr() masks the upper 32 bits for ILP32
 *      callers before the value is cast to a kernel pointer, preventing a
 *      crafted high-bits value from being passed as a kernel address to
 *      copyin(), which rejects kernel VAs when called from a user context.
 *
 *   2. td->td_sa.args[2]  (original user ioctl data pointer)
 *
 *      kern_ioctl() replaces the user-space data pointer with a kernel
 *      buffer before calling d_ioctl / fo_ioctl.  We fish the original
 *      out of td->td_sa.args[2] for code paths that must copyout directly
 *      to user space on non-zero returns (EOWNERDEAD, index write-back in
 *      wait_any / wait_all), because kern_ioctl discards the kernel buffer
 *      on error.
 *
 *      freebsd32_ioctl(2) zero-extends the 32-bit argument when storing it
 *      in td->td_sa.args[2].  ntsync_udata_ptr() applies an explicit
 *      uint32_t mask for ILP32 callers as a defensive measure against any
 *      platform that might sign-extend rather than zero-extend 32-bit
 *      register values.
 */

/*
 * Compile-time ABI-safety assertions.
 *
 * Unconditional: a non-compat build must also catch layout regressions.
 * If a size changes the ioctl command codes will silently diverge between
 * 32-bit and 64-bit Wine binaries.  Fix the struct (add explicit padding or
 * keep fixed-width fields) and update the expected sizes here.
 */
_Static_assert(sizeof(struct ntsync_sem_args)   ==  8,
    "ntsync: ntsync_sem_args size changed -- "
    "ioctl numbers will diverge between ILP32 and LP64 ABIs");
_Static_assert(sizeof(struct ntsync_mutex_args) ==  8,
    "ntsync: ntsync_mutex_args size changed -- "
    "ioctl numbers will diverge between ILP32 and LP64 ABIs");
_Static_assert(sizeof(struct ntsync_event_args) ==  8,
    "ntsync: ntsync_event_args size changed -- "
    "ioctl numbers will diverge between ILP32 and LP64 ABIs");
_Static_assert(sizeof(struct ntsync_wait_args)  == 40,
    "ntsync: ntsync_wait_args size changed -- "
    "ioctl numbers will diverge between ILP32 and LP64 ABIs");

/*
 * ntsync_proc_is_32bit()
 *
 * Returns true when the calling thread belongs to a 32-bit (ILP32) process
 * running under the COMPAT_FREEBSD32 personality.  This is the single
 * runtime predicate used by ntsync_objs_uptr() and ntsync_udata_ptr() to
 * gate ILP32-specific behaviour.  Confining the #ifdef to one place keeps
 * the remaining helpers unconditionally readable.
 */
static inline bool
ntsync_proc_is_32bit(struct thread *td)
{
#ifdef COMPAT_FREEBSD32
    return (SV_PROC_FLAG(td->td_proc, SV_ILP32) != 0);
#else
    (void)td;
    return (false);
#endif
}

/*
 * ntsync_udata_ptr()
 *
 * Recover the original user-space ioctl data pointer from the saved syscall
 * arguments.  kern_ioctl() replaces the user-space data pointer with a
 * kernel buffer before calling d_ioctl / fo_ioctl; we recover the original
 * from td->td_sa.args[2] so that operations that must return data on
 * non-zero returns (EOWNERDEAD, index write-back) can copyout directly to
 * user space — kern_ioctl discards the kernel buffer on error returns.
 *
 * For ILP32 (WOW64) callers freebsd32_ioctl(2) zero-extends the 32-bit
 * argument into td->td_sa.args[2]; the explicit uint32_t cast is a
 * defensive measure against platforms that might sign-extend instead.
 */
static inline void *
ntsync_udata_ptr(struct thread *td)
{
    if (ntsync_proc_is_32bit(td))
        return ((void *)(uintptr_t)(uint32_t)td->td_sa.args[2]);
    return ((void *)(uintptr_t)td->td_sa.args[2]);
}

/*
 * ntsync_objs_uptr()
 *
 * Safely extract the user-space objs array pointer from ntsync_wait_args.
 * The objs field is __u64 to hold a full 64-bit address from a native LP64
 * caller.  A 32-bit (WOW64) caller stores a 32-bit pointer in the low half;
 * the upper 32 bits should be zero but cannot be trusted from an adversarial
 * process.  Masking before the cast prevents a crafted high-bits value from
 * being passed as a kernel address to copyin(), which rejects kernel VAs when
 * called from a user context.  Without the mask a value above KERNBASE would
 * cause copyin() to return EFAULT, but the correct rejection happens for the
 * right reason rather than by accident.
 *
 * For LP64 callers the raw value is used unchanged; applying the mask would
 * incorrectly truncate a legitimate 64-bit user address.
 */
static inline void *
ntsync_objs_uptr(struct thread *td, const struct ntsync_wait_args *args)
{
    uint64_t raw = args->objs;

    if (ntsync_proc_is_32bit(td))
        raw &= UINT64_C(0x00000000ffffffff);
    return ((void *)(uintptr_t)raw);
}

#ifndef __u32
#define __u32 uint32_t
#endif
#ifndef __u64
#define __u64 uint64_t
#endif

#define NTSYNC_NAME "ntsync"

MALLOC_DEFINE(M_NTSYNC, "ntsync", "NT synchronization primitives");

/*
 * UMA zone for ntsync_q allocations.
 *
 * Every WAIT_ANY/WAIT_ALL call allocates one ntsync_q (fixed header +
 * up to NTSYNC_MAX_WAIT_COUNT+1 entries).  Under a running game this
 * happens thousands of times per second; using a UMA slab zone instead
 * of malloc(M_NTSYNC) eliminates allocator round-trips and the per-call
 * mtx_init/cv_init overhead visible in the malloc path.
 *
 * All allocations are capped at NTSYNC_Q_MAX_SIZE regardless of the
 * actual object count; the modest memory overhead (≤65 entries per slab
 * item) is the correct trade-off for a synchronisation hot path.
 */
#define NTSYNC_Q_MAX_SIZE \
    (sizeof(struct ntsync_q) + \
     (NTSYNC_MAX_WAIT_COUNT + 1) * sizeof(struct ntsync_q_entry))

static uma_zone_t ntsync_q_zone;

/*
 * Count of ntsync_q objects currently allocated from ntsync_q_zone.
 * Incremented by ntsync_setup_wait immediately after uma_zalloc and
 * decremented before every uma_zfree.  MOD_UNLOAD polls this counter
 * before calling uma_zdestroy to avoid destroying the zone while threads
 * are still sleeping inside ntsync_schedule with a live ntsync_q pointer.
 *
 * Implemented as a per-CPU counter (counter(9)) to avoid a global
 * cache-line bounce on every WAIT_ANY / WAIT_ALL call.  Under a running
 * game the hot path issues thousands of increments and decrements per
 * second across multiple CPU cores; a plain atomic_add_int serialises
 * those onto a single shared cache line.  counter_u64_add() uses
 * CPU-local storage so each core updates its own slot with no coherence
 * traffic.  counter_u64_fetch() aggregates all slots and is only called
 * from MOD_UNLOAD, which is a rare cold path.
 */
static counter_u64_t ntsync_active_q_count;
static counter_u64_t ntsync_obj_count;

/* Compute a user-space pointer to a field inside a user-supplied struct.
 * Usage: USER_FIELD_PTR(uarg, struct ntsync_wait_args, index)
 * Returns: void * user-space address of that field.
 */
#define USER_FIELD_PTR(u, type, field) \
    ((void *)((uintptr_t)(u) + offsetof(type, field)))

/*
 * ntsync_kern_copyin / ntsync_kern_copyout
 *
 * Fast helpers for the common case where both src and dst are kernel
 * buffers — specifically, the caddr_t 'data' buffer that kern_ioctl
 * allocates and passes into d_ioctl / fo_ioctl.  Using these instead
 * of ntsync_safe_copyin/copyout avoids a user-context copyin() attempt
 * on a buffer that is already in kernel memory.
 *
 * ONLY use these when 'uarg' is known to be a kernel pointer (i.e. the
 * data argument received directly from kern_ioctl).  Continue to use
 * copyin/copyout or ntsync_safe_copyin for genuine user-space pointers
 * such as args->objs in ntsync_setup_wait.
 */
static inline int
ntsync_kern_copyin(const void *src, void *dst, size_t len)
{
    if (len == 0)
        return (0);
    if (src == NULL || dst == NULL)
        return (EFAULT);
    memcpy(dst, src, len);
    return (0);
}

static inline int
ntsync_kern_copyout(const void *src, void *dst, size_t len)
{
    if (len == 0)
        return (0);
    if (src == NULL || dst == NULL)
        return (EFAULT);
    memcpy(dst, src, len);
    return (0);
}

/* Helper: copy a single field value from a kernel ioctl buffer to another
 * kernel ioctl buffer (both are kernel-VA; no user-space pointer involved).
 * Returns 0 on success, EFAULT on failure.
 */
static inline int
ntsync_copyout_field(void *user_base, size_t field_offset, const void *kbuf, size_t len)
{
    void *user_ptr = (void *)((uintptr_t)user_base + field_offset);
    return ntsync_kern_copyout(kbuf, user_ptr, len);
}

enum ntsync_type {
    NTSYNC_TYPE_SEM,
    NTSYNC_TYPE_MUTEX,
    NTSYNC_TYPE_EVENT,
};

struct ntsync_device;
struct ntsync_obj;
struct ntsync_q;

struct ntsync_q_entry {
    TAILQ_ENTRY(ntsync_q_entry) link;
    struct ntsync_q *q;
    struct ntsync_obj *obj;
    __u32 index;
};

struct ntsync_q {
    struct thread *td;
    __u32 owner;
    int signaled; /* -1 = none, >=0 index */
    bool all;
    bool ownerdead;
    __u32 count; /* number of normal objects (excluding alert) */
    struct mtx lock;
    struct cv cv;
    /* entries[count (+1 if alert)] */
    /* New: aborted flag to indicate waiter should abort due to object deletion */
    bool aborted;
    struct ntsync_q_entry entries[];
};

struct ntsync_obj {
    struct mtx mtx;
    enum ntsync_type type;
    struct ntsync_device *dev;
    union {
        struct {
            __u32 count;
            __u32 max;
        } sem;
        struct {
            __u32 count;
            pid_t owner;
            bool ownerdead;
        } mutex;
        struct {
            bool manual;
            bool signaled;
        } event;
    } u;
    TAILQ_HEAD(, ntsync_q_entry) any_waiters;
    TAILQ_HEAD(, ntsync_q_entry) all_waiters;
    int all_hint;
    struct selinfo selinfo;
    struct knlist knlist;
    /*
     * notify_hint: set to 1 (sticky, never cleared) the first time a
     * poll(2) or kqueue(2) watcher registers on this object.  Used to
     * gate ntsync_obj_notify: Wine never calls poll/kqueue on ntsync fds
     * during normal gameplay, so both selwakeup() and KNOTE_LOCKED() can
     * be skipped entirely on the hot signal path when no watcher has ever
     * registered.  A false positive (hint=1 but no active watcher) is safe
     * — selwakeup/KNOTE_LOCKED already have cheap internal empty-list guards.
     * A false negative is impossible: hint is set before selrecord/knlist_add.
     */
    int notify_hint;
    int refcnt;
    /* New: deleted flag to notify waiters that object is being destroyed/closed */
    int deleted;

    /* waiter drain fields */
    int active_waiters;
    struct cv waiters_cv;

    /* debug id to detect reuse */
    uint64_t id;
};

struct ntsync_device {
    struct mtx wait_all_mtx;
    struct cdev *cdev;
    /*
     * Device reference count.  Initialised to 1 (the module's own reference)
     * in MOD_LOAD.  Each live ntsync_obj holds an additional reference
     * (taken in ntsync_alloc_obj, released via ntsync_dev_drop in
     * ntsync_obj_unref).  The device's mutex and memory are freed only when
     * the count reaches zero — preventing the MOD_UNLOAD path from freeing
     * the device while object-fd ioctls still access wait_all_mtx.
     */
    int refcnt;
};

static struct ntsync_device *ntsync_dev;

/* Forward declarations */
static d_open_t ntsync_open;
static d_close_t ntsync_close;
static d_ioctl_t ntsync_ioctl;

static struct cdevsw ntsync_cdevsw = {
    .d_version = D_VERSION,
    .d_open = ntsync_open,
    .d_close = ntsync_close,
    .d_ioctl = ntsync_ioctl,
    .d_name = NTSYNC_NAME,
};

/* Object fileops */
static fo_close_t ntsync_obj_close;
static fo_ioctl_t ntsync_obj_ioctl;
static fo_poll_t ntsync_obj_poll;
static fo_kqfilter_t ntsync_obj_kqfilter;

/*
 * fileops stubs — all function pointers in struct fileops MUST be non-NULL.
 * FreeBSD's VFS dispatch calls them directly; a NULL pointer causes an
 * immediate kernel panic.  The invfo_* family covers most cases, but
 * invfo_stat and invfo_seek do not exist in all FreeBSD 13/14 kernel trees,
 * and fo_fill_kinfo needs a real struct kinfo_file (from sys/user.h).
 * We provide minimal custom stubs for each missing symbol.
 */

/*
 * fstat(2) on an ntsync fd.
 *
 * Wine calls fstat() on file descriptors to verify they are legitimate
 * ntsync objects before issuing NTSYNC_IOC_WAIT_ANY / ALL ioctls. 
 * Wine's is_ntsync_fd() function explicitly checks two things:
 * 1. The file must be a character device (S_ISCHR).
 * 2. The st_rdev must exactly match the device ID of /dev/ntsync.
 *
 * We populate a minimal struct stat with st_mode = S_IFCHR and extract
 * the real device ID using dev2udev() so Wine's validation passes and
 * it doesn't fall back to the legacy wineserver for synchronization.
 */
static int
ntsync_obj_stat(struct file *fp, struct stat *sb,
    struct ucred *active_cred)
{
    struct ntsync_obj *obj = fp->f_data;

    bzero(sb, sizeof(*sb));
    sb->st_mode = S_IFCHR;

    /*
     * Wine relies on st_rdev matching the /dev/ntsync device ID.
     *
     * Snapshot obj->dev->cdev into a local before calling dev2udev().
     * During MOD_UNLOAD the sequence is: destroy_dev() then
     * ntsync_dev->cdev = NULL.  Without the snapshot, a concurrent
     * fstat() can observe cdev != NULL in the check and then receive
     * NULL (or a freed pointer) in dev2udev(), panicking the kernel.
     *
     * On all supported FreeBSD architectures (amd64, arm64) a pointer
     * load is naturally atomic, so the snapshot races safely with the
     * NULL store.  If we capture the old non-NULL pointer, dev2udev()
     * is still safe: destroy_dev() only queues the cdev for deferred
     * teardown; the underlying devfs inode (si_priv) is not freed until
     * all concurrent accesses have completed.
     */
    if (obj != NULL && obj->dev != NULL) {
        struct cdev *snap = obj->dev->cdev;   /* atomic pointer load */
        if (snap != NULL)
            sb->st_rdev = dev2udev(snap);
    }

    /* Set a non-zero st_dev to look like a real filesystem entry */
    sb->st_dev = 1;

    /* Optional but good practice for character devices */
    sb->st_blksize = PAGE_SIZE;

    return (0);
}

/* lseek(2): ntsync fds are not seekable. */
static int
ntsync_obj_seek(struct file *fp, off_t offset, int whence,
    struct thread *td)
{
    return (ESPIPE);
}

/* fo_fill_kinfo: called by procstat(1), ps(1), top(1) while iterating all
 * open fds. Must return 0 — a non-zero return aborts procstat's fd walk.
 * Set kf_type to KF_TYPE_UNKNOWN so the fd shows up as "?" rather than
 * crashing the tool. */
static int
ntsync_fill_kinfo(struct file *fp, struct kinfo_file *kif,
    struct filedesc *fdp)
{
    kif->kf_type = KF_TYPE_UNKNOWN;
    return (0);
}

static struct fileops ntsync_obj_fileops = {
    /*
     * All slots must be filled. NULL slots → kernel panic on any syscall
     * that dispatches through the corresponding field.
     */
    .fo_read      = invfo_rdwr,
    .fo_write     = invfo_rdwr,
    .fo_truncate  = invfo_truncate,
    .fo_ioctl     = ntsync_obj_ioctl,
    .fo_poll      = ntsync_obj_poll,
    .fo_kqfilter  = ntsync_obj_kqfilter,
    .fo_stat      = ntsync_obj_stat,
    .fo_close     = ntsync_obj_close,
    .fo_chmod     = invfo_chmod,
    .fo_chown     = invfo_chown,
    .fo_sendfile  = invfo_sendfile,
    .fo_seek      = ntsync_obj_seek,
    .fo_fill_kinfo = ntsync_fill_kinfo,
    .fo_flags     = DFLAG_PASSABLE,
};

/* kqueue filter ops */
static int ntsync_kqfilter_read(struct knote *kn, long hint);
static void ntsync_kqfilter_detach(struct knote *kn);

static struct filterops ntsync_kqread_filtops = {
    .f_isfd = 1,
    .f_detach = ntsync_kqfilter_detach,
    .f_event = ntsync_kqfilter_read,
};

/* Helper prototype */
static void ntsync_q_signal_deleted(struct ntsync_q *q, __u32 idx);

static void
ntsync_q_signal_deleted(struct ntsync_q *q, __u32 idx)
{
    /*
     * q->lock must be held when reading/writing q->signaled and q->aborted,
     * and when calling cv_signal.  FreeBSD requires the condvar's associated
     * mutex to be held when signalling.  The associated mutex for q->cv is
     * q->lock, as established by cv_wait_sig and cv_timedwait_sig_sbt in
     * ntsync_schedule.  Callers hold obj->mtx; the ordering obj->mtx ->
     * q->lock matches all ntsync_try_wake_any_* functions.
     *
     * cv_signal (not cv_broadcast): each ntsync_q has exactly one sleeping
     * thread (q->td, set in ntsync_q_init and never changed).  cv_signal is
     * correct and marginally cheaper because it does not need to drain every
     * sleeper from the sleep queue.
     */
    mtx_lock(&q->lock);
    if (q->signaled == -1) {
        q->signaled = idx;
        q->aborted = 1;
        cv_signal(&q->cv);
    }
    mtx_unlock(&q->lock);
}

/* ---------------------- Utility: safe copyin ------------------------- */

/*
 * ntsync_safe_copyin: copy 'len' bytes from the user-space address 'uaddr'
 * into the kernel buffer 'kbuf'.
 *
 * This wrapper is used exclusively for user-supplied pointers (e.g.
 * ntsync_wait_args::objs) and must NEVER fall back to a kernel-VA memcpy.
 *
 * The old implementation contained a pmap_kextract + memcpy fallback that
 * was reachable when a 64-bit caller crafted an objs pointer that mapped to
 * a physically-backed kernel VA: copyin() would fail (kernel addr in user
 * context), pmap_kextract would confirm a valid PA, and memcpy would then
 * read from kernel memory into the fd array — an unintended kernel memory
 * read from user-controlled addresses.  The fallback has been removed.
 *
 * Callers that need to copy from a kernel buffer (e.g. the kern_ioctl data
 * argument) must use ntsync_kern_copyin() instead.
 */
static int
ntsync_safe_copyin(const void *uaddr, void *kbuf, size_t len)
{
    if (len == 0)
        return (0);
    if (uaddr == NULL || kbuf == NULL)
        return (EFAULT);
    return (copyin(uaddr, kbuf, len));
}


/* ---------------------- Object lifetime helpers ---------------------- */

/* Forward declarations to avoid implicit declarations under C99+ */
static void ntsync_try_wake_any_obj(struct ntsync_obj *obj);
static void ntsync_try_wake_all_obj(struct ntsync_device *dev, struct ntsync_obj *obj);
static void ntsync_obj_notify(struct ntsync_obj *obj);
static void ntsync_dev_drop(struct ntsync_device *dev);

static void
ntsync_obj_ref(struct ntsync_obj *obj)
{
#ifdef NTSYNC_DEBUG
    int new = atomic_fetchadd_int(&obj->refcnt, 1) + 1;
    NTSYNC_DPRINTF("ntsync: ref obj %p id=%ju refcnt=%d\n", obj, (uintmax_t)obj->id, new);
#else
    atomic_fetchadd_int(&obj->refcnt, 1);
#endif
}

static void
ntsync_obj_unref(struct ntsync_obj *obj)
{
    if (atomic_fetchadd_int(&obj->refcnt, -1) == 1) {
        struct ntsync_device *dev = obj->dev;

        if (dev != NULL)
            mtx_lock(&dev->wait_all_mtx);
        mtx_lock(&obj->mtx);

        NTSYNC_DPRINTF("ntsync: unref obj %p id=%ju before_free deleted=%d active_waiters=%d\n",
               obj, (uintmax_t)obj->id, obj->deleted, obj->active_waiters);
        KASSERT(obj->active_waiters == 0,
            ("ntsync: freeing object %p id=%ju with %d active waiters -- "
             "refcount / drain invariant broken",
             obj, (uintmax_t)obj->id, obj->active_waiters));
        /* mark deleted so wake helpers abort waiters rather than try to consume state */
        obj->deleted = 1;
        NTSYNC_DPRINTF("ntsync: obj_unref %p id=%ju deleted=1, waking waiters\n",
               obj, (uintmax_t)obj->id);

        /* (b) Log active_waiters and a concise any_waiters summary (mirror of obj_close) */
        NTSYNC_DPRINTF("ntsync: obj_unref %p active_waiters=%d\n", obj, obj->active_waiters);
        if (TAILQ_EMPTY(&obj->any_waiters)) {
            NTSYNC_DPRINTF("ntsync: obj %p any_waiters EMPTY\n", obj);
        } else {
            struct ntsync_q_entry *e;
            NTSYNC_DPRINTF("ntsync: obj %p any_waiters list:\n", obj);
            TAILQ_FOREACH(e, &obj->any_waiters, link) {
                NTSYNC_DPRINTF("ntsync:   any_waiters entry %p -> q=%p idx=%u obj=%p\n",
                       e, (void *)e->q, e->index, (void *)e->obj);
            }
        }

        if (dev != NULL) {
            ntsync_try_wake_all_obj(dev, obj);
        }
        ntsync_try_wake_any_obj(obj);
        ntsync_obj_notify(obj);
        mtx_unlock(&obj->mtx);
        if (dev != NULL)
            mtx_unlock(&dev->wait_all_mtx);
        knlist_destroy(&obj->knlist);
        seldrain(&obj->selinfo);
        cv_destroy(&obj->waiters_cv);
        mtx_destroy(&obj->mtx);
        free(obj, M_NTSYNC);
        /*
         * Drop the device reference this object was holding.  If this was
         * the last object and MOD_UNLOAD has already called destroy_dev()
         * and ntsync_dev_drop(), this will be the final drop that frees
         * the device structure and destroys wait_all_mtx.
         */
        if (dev != NULL)
            ntsync_dev_drop(dev);
    }
}

/*
 * ntsync_obj_drop()
 *
 * In debug builds: a full function that logs the call site and checks for NULL.
 * In production builds: expands directly to ntsync_obj_unref(), eliminating the
 * function call overhead, the const char* argument push, and the NULL-check branch
 * on every hot-path callsite (18 total, many inside per-object teardown loops that
 * run thousands of times per second under a running game).
 */
#ifdef NTSYNC_DEBUG
static void
ntsync_obj_drop(struct ntsync_obj *obj, const char *where)
{
    if (obj == NULL) {
        printf("ntsync: drop called with NULL (%s)\n", where);
        return;
    }
    NTSYNC_DPRINTF("ntsync: drop %s obj=%p id=%ju deleted=%d active_waiters=%d\n",
           where, obj, (uintmax_t)obj->id, obj->deleted, obj->active_waiters);
    ntsync_obj_unref(obj);
}
#else
#define ntsync_obj_drop(obj, where)  ntsync_obj_unref(obj)
#endif

/*
 * ntsync_dev_drop()
 *
 * Drop a reference on the ntsync_device.  When the count reaches zero all
 * live ntsync_obj instances have already been freed and MOD_UNLOAD has called
 * destroy_dev(), so it is now safe to destroy the wait_all mutex and free the
 * device structure.
 */
static void
ntsync_dev_drop(struct ntsync_device *dev)
{
    if (dev == NULL)
        return;
    if (atomic_fetchadd_int(&dev->refcnt, -1) == 1) {
        mtx_destroy(&dev->wait_all_mtx);
        free(dev, M_NTSYNC);
    }
}

/* ---------------------- Signaled checks / notify --------------------- */

static bool
ntsync_is_signaled_owner(struct ntsync_obj *obj, __u32 owner)
{
    bool ret = false;

    mtx_assert(&obj->mtx, MA_OWNED);
    switch (obj->type) {
    case NTSYNC_TYPE_SEM:
        ret = (obj->u.sem.count > 0);
        break;
    case NTSYNC_TYPE_MUTEX:
        if (obj->u.mutex.owner && obj->u.mutex.owner != (pid_t)owner)
            ret = false;
        else
            ret = (obj->u.mutex.count < UINT_MAX);
        break;
    case NTSYNC_TYPE_EVENT:
        ret = obj->u.event.signaled;
        break;
    default:
        ret = false;
        break;
    }
    return ret;
}

static void
ntsync_obj_notify(struct ntsync_obj *obj)
{
    /*
     * Fast path: if no poll(2)/kqueue(2) watcher has ever registered on
     * this object, skip both selwakeup and KNOTE_LOCKED entirely.
     * notify_hint is set (sticky) in ntsync_obj_poll and ntsync_obj_kqfilter
     * before selrecord/knlist_add, so it is always 1 when a watcher exists.
     * In Wine gaming workloads this branch is never taken and the body is
     * never executed, eliminating two function calls per signal operation.
     */
    if (__predict_false(obj->notify_hint)) {
        selwakeup(&obj->selinfo);
        /* obj->mtx is always held by callers; use the locked variant. */
        KNOTE_LOCKED(&obj->knlist, 0);
    }
}

/* ---------------------- Wait queue helpers --------------------------- */

static void
ntsync_q_init(struct ntsync_q *q, __u32 owner, bool all, __u32 count,
    __u32 total_count)
{
    __u32 i;

    q->td = curthread;
    q->owner = owner;
    q->signaled = -1;
    q->all = all;
    q->ownerdead = false;
    q->count = count;
    q->aborted = false;
    /*
     * mtx and cv are pre-initialised by the UMA init callback (ntsync_q_uma_init)
     * and persist across zone alloc/free cycles — do NOT call mtx_init/cv_init here.
     *
     * M_ZERO is not used on uma_zalloc (it would zero and corrupt the live mtx/cv),
     * so we must explicitly reset the fields that ntsync_setup_wait's error-path
     * cleanup reads before assignment.  Specifically, entry->q is tested against the
     * current 'q' pointer to detect entries that were actually inserted; stale
     * non-NULL values from a previous use of this slab slot would cause false
     * positives and corrupt the waiter lists.
     *
     * Only zero the slots that setup_wait will actually touch (total_count =
     * count + 1 if alert is set, else count).  The remaining 64-count slots
     * are never accessed by any cleanup path and need not be zeroed.
     */
    for (i = 0; i < total_count; i++)
        q->entries[i].q = NULL;
}

/* ---------------------- Object allocation / fd ----------------------- */

static struct ntsync_obj *
ntsync_alloc_obj(struct ntsync_device *dev, enum ntsync_type type)
{
    struct ntsync_obj *obj;

    obj = malloc(sizeof(*obj), M_NTSYNC, M_WAITOK | M_ZERO);
    mtx_init(&obj->mtx, "ntsync_obj", NULL, MTX_DEF);
    obj->type = type;
    obj->dev = dev;
    TAILQ_INIT(&obj->any_waiters);
    TAILQ_INIT(&obj->all_waiters);
    obj->all_hint = 0;
    obj->refcnt = 1;
    obj->deleted = 0;
    obj->active_waiters = 0;
    cv_init(&obj->waiters_cv, "ntsync_waiters_cv");
    knlist_init_mtx(&obj->knlist, &obj->mtx);
    /* Each live object holds a reference on its device to prevent MOD_UNLOAD
     * from freeing ntsync_dev while object-fd ioctls still access wait_all_mtx.
     */
    if (dev != NULL)
        atomic_fetchadd_int(&dev->refcnt, 1);
    /*
     * Increment the global object counter. In NTSYNC_DEBUG mode, use
     * the aggregate count as a non-unique trace marker to identify
     * objects without the cache-line bouncing of a global atomic.
     */
#ifdef NTSYNC_DEBUG
    counter_u64_add(ntsync_obj_count, 1);
    obj->id = counter_u64_fetch(ntsync_obj_count);
#else
    obj->id = 0;
#endif
    NTSYNC_DPRINTF("ntsync: alloc obj %p id=%ju\n", obj, (uintmax_t)obj->id);
    return obj;
}

/*
 * Create a new file descriptor for an object.
 * Uses falloc() which allocates and installs the descriptor.
 * Returns 0 on success with *fdp set, or a positive errno on failure.
 *
 * The previous version returned the fd directly (>= 0) or a positive errno
 * on failure, but all callers checked `if (fd < 0)` — a test that can never
 * be true for errno values (ENOMEM=12, EMFILE=24, ...).  Every falloc()
 * failure was silently swallowed and the errno value was stored as an fd.
 */
static int
ntsync_obj_get_fd(struct thread *td, struct ntsync_obj *obj, int *fdp)
{
    struct file *fp;
    int fd, error;

    error = falloc(td, &fp, &fd, 0);
    if (error != 0)
        return (error);

    fp->f_type = DTYPE_DEV;
    fp->f_ops = &ntsync_obj_fileops;
    fp->f_data = obj;
    fp->f_flag = FREAD | FWRITE;
    ntsync_obj_ref(obj);
    fdrop(fp, td);
    *fdp = fd;
    return (0);
}

/*
 * Get an object from a user FD.
 * Increments obj refcount; caller must ntsync_obj_drop() or ntsync_obj_unref().
 */
static int
ntsync_get_obj(struct thread *td, int fd, struct ntsync_device *dev, struct ntsync_obj **out)
{
    struct file *fp;
    cap_rights_t rights;
    int error;

    cap_rights_init(&rights, CAP_IOCTL);
    error = fget(td, fd, &rights, &fp);
    if (error != 0)
        return (error);

    if (fp->f_ops != &ntsync_obj_fileops) {
        fdrop(fp, td);
        return (EINVAL);
    }

    struct ntsync_obj *obj = fp->f_data;
    if (obj->dev != dev) {
        fdrop(fp, td);
        return (EINVAL);
    }

    ntsync_obj_ref(obj);
    fdrop(fp, td);
    *out = obj;
    return (0);
}

/* ---------------------- Core state transitions ----------------------- */

static int
ntsync_sem_release_state(struct ntsync_obj *sem, __u32 count)
{
    __u64 sum;

    mtx_assert(&sem->mtx, MA_OWNED);
    sum = (__u64)sem->u.sem.count + (__u64)count;
    if (sum > sem->u.sem.max)
        return (EOVERFLOW);
    sem->u.sem.count = (__u32)sum;
    return (0);
}

static int
ntsync_mutex_unlock_state(struct ntsync_obj *mutex, const struct ntsync_mutex_args *args)
{
    mtx_assert(&mutex->mtx, MA_OWNED);
    if (mutex->u.mutex.owner != (pid_t)args->owner)
        return (EPERM);
    /*
     * Guard against a double-unlock from userspace.  Without this check
     * a count of 0 would pre-decrement to UINT32_MAX, permanently stalling
     * ntsync_try_wake_any_mutex (which breaks out of the waiter loop when
     * count == UINT_MAX) and making ntsync_is_signaled_owner always return
     * false, deadlocking every subsequent waiter on this mutex.
     */
    if (mutex->u.mutex.count == 0)
        return (EPERM);
    if (--mutex->u.mutex.count == 0)
        mutex->u.mutex.owner = 0;
    return (0);
}

static int
ntsync_mutex_kill_state(struct ntsync_obj *mutex, __u32 owner)
{
    mtx_assert(&mutex->mtx, MA_OWNED);
    if (mutex->u.mutex.owner != (pid_t)owner)
        return (EPERM);
    mutex->u.mutex.ownerdead = true;
    mutex->u.mutex.owner = 0;
    mutex->u.mutex.count = 0;
    return (0);
}

/* ---------------------- Waking waiters -------------------------------- */

static void ntsync_try_wake_any_sem(struct ntsync_obj *sem);
static void ntsync_try_wake_any_mutex(struct ntsync_obj *mutex);
static void ntsync_try_wake_any_event(struct ntsync_obj *event);

static void
ntsync_try_wake_any_sem(struct ntsync_obj *sem)
{
    struct ntsync_q_entry *entry;

    mtx_assert(&sem->mtx, MA_OWNED);

    /* Diagnostic: print object state before iterating */
    NTSYNC_DPRINTF("ntsync: try_wake_any_sem obj=%p id=%ju state: sem.count=%u sem.max=%u deleted=%d active_waiters=%d\n",
           sem, (uintmax_t)sem->id, sem->u.sem.count, sem->u.sem.max, sem->deleted, sem->active_waiters);

    /* Diagnostic: count any_waiters before attempting wake */
#ifdef NTSYNC_DEBUG
    {
        struct ntsync_q_entry *e;
        int cnt = 0;
        TAILQ_FOREACH(e, &sem->any_waiters, link)
            cnt++;
        NTSYNC_DPRINTF("ntsync: try_wake_any_sem obj=%p id=%ju any_waiters_count=%d\n",
               sem, (uintmax_t)sem->id, cnt);
    }
#endif
    /*
     * Hoist the deleted check: sem->deleted is stable under sem->mtx (held
     * throughout this function).  Splitting into two loops eliminates one
     * branch per iteration on the non-deleted fast path — the only path
     * taken during normal game operation.
     */
    if (sem->deleted) {
        TAILQ_FOREACH(entry, &sem->any_waiters, link) {
            struct ntsync_q *q = entry->q;
            NTSYNC_DPRINTF("ntsync: try_wake_any_sem inspecting entry q=%p index=%u (obj=%p id=%ju)\n",
                   entry->q, entry->index, sem, (uintmax_t)sem->id);
            mtx_lock(&q->lock);
            if (q->signaled == -1) {
                q->aborted = true;
                q->signaled = entry->index;
                NTSYNC_DPRINTF("ntsync: wake_abort sem obj=%p id=%ju -> q=%p (waker tid=%d) signaled=%d aborted=1\n",
                       sem, (uintmax_t)sem->id, q, curthread->td_tid, q->signaled);
                cv_signal(&q->cv);
            }
            mtx_unlock(&q->lock);
        }
        return;
    }
    TAILQ_FOREACH(entry, &sem->any_waiters, link) {
        struct ntsync_q *q = entry->q;
        NTSYNC_DPRINTF("ntsync: try_wake_any_sem inspecting entry q=%p index=%u (obj=%p id=%ju)\n",
               entry->q, entry->index, sem, (uintmax_t)sem->id);
        if (sem->u.sem.count == 0)
            break;
        mtx_lock(&q->lock);
        if (q->signaled == -1) {
            q->signaled = entry->index;
            sem->u.sem.count--;
            NTSYNC_DPRINTF("ntsync: wake_any sem obj=%p id=%ju -> q=%p (waker tid=%d) signaled=%d\n",
                   sem, (uintmax_t)sem->id, q, curthread->td_tid, q->signaled);
            cv_signal(&q->cv);
        }
        mtx_unlock(&q->lock);
    }
}

static void
ntsync_try_wake_any_mutex(struct ntsync_obj *mutex)
{
    struct ntsync_q_entry *entry;

    mtx_assert(&mutex->mtx, MA_OWNED);

    /* Diagnostic: print object state before iterating */
    NTSYNC_DPRINTF("ntsync: try_wake_any_mutex obj=%p id=%ju state: mutex.count=%u owner=%d ownerdead=%d deleted=%d active_waiters=%d\n",
           mutex, (uintmax_t)mutex->id, mutex->u.mutex.count, mutex->u.mutex.owner, mutex->u.mutex.ownerdead, mutex->deleted, mutex->active_waiters);

    /* Diagnostic: count any_waiters before attempting wake */
#ifdef NTSYNC_DEBUG
    {
        struct ntsync_q_entry *e;
        int cnt = 0;
        TAILQ_FOREACH(e, &mutex->any_waiters, link)
            cnt++;
        NTSYNC_DPRINTF("ntsync: try_wake_any_mutex obj=%p id=%ju any_waiters_count=%d\n",
               mutex, (uintmax_t)mutex->id, cnt);
    }
#endif
    /*
     * Hoist the deleted check: mutex->deleted is stable under mutex->mtx.
     * Two separate loops avoid a per-iteration branch on the hot non-deleted path.
     */
    if (mutex->deleted) {
        TAILQ_FOREACH(entry, &mutex->any_waiters, link) {
            struct ntsync_q *q = entry->q;
            NTSYNC_DPRINTF("ntsync: try_wake_any_mutex inspecting entry q=%p index=%u (obj=%p id=%ju)\n",
                   entry->q, entry->index, mutex, (uintmax_t)mutex->id);
            mtx_lock(&q->lock);
            if (q->signaled == -1) {
                q->aborted = true;
                q->signaled = entry->index;
                NTSYNC_DPRINTF("ntsync: wake_abort mutex obj=%p id=%ju -> q=%p (waker tid=%d) signaled=%d aborted=1\n",
                       mutex, (uintmax_t)mutex->id, q, curthread->td_tid, q->signaled);
                cv_signal(&q->cv);
            }
            mtx_unlock(&q->lock);
        }
        return;
    }
    TAILQ_FOREACH(entry, &mutex->any_waiters, link) {
        struct ntsync_q *q = entry->q;
        NTSYNC_DPRINTF("ntsync: try_wake_any_mutex inspecting entry q=%p index=%u (obj=%p id=%ju)\n",
               entry->q, entry->index, mutex, (uintmax_t)mutex->id);
        if (mutex->u.mutex.count == UINT_MAX)
            break;
        if (mutex->u.mutex.owner && mutex->u.mutex.owner != (pid_t)q->owner)
            continue;
        mtx_lock(&q->lock);
        if (q->signaled == -1) {
            if (mutex->u.mutex.ownerdead)
                q->ownerdead = true;
            mutex->u.mutex.ownerdead = false;
            mutex->u.mutex.count++;
            mutex->u.mutex.owner = (pid_t)q->owner;
            q->signaled = entry->index;
            NTSYNC_DPRINTF("ntsync: wake_any mutex obj=%p id=%ju -> q=%p (waker tid=%d) signaled=%d\n",
                   mutex, (uintmax_t)mutex->id, q, curthread->td_tid, q->signaled);
            cv_signal(&q->cv);
        }
        mtx_unlock(&q->lock);
    }
}

static void
ntsync_try_wake_any_event(struct ntsync_obj *event)
{
    struct ntsync_q_entry *entry;

    mtx_assert(&event->mtx, MA_OWNED);

    /* Diagnostic: print object state before iterating */
    NTSYNC_DPRINTF("ntsync: try_wake_any_event obj=%p id=%ju state: event.signaled=%d manual=%d deleted=%d active_waiters=%d\n",
           event, (uintmax_t)event->id, event->u.event.signaled, event->u.event.manual, event->deleted, event->active_waiters);

    /* Diagnostic: count any_waiters before attempting wake */
#ifdef NTSYNC_DEBUG
    {
        struct ntsync_q_entry *e;
        int cnt = 0;
        TAILQ_FOREACH(e, &event->any_waiters, link)
            cnt++;
        NTSYNC_DPRINTF("ntsync: try_wake_any_event obj=%p id=%ju any_waiters_count=%d\n",
               event, (uintmax_t)event->id, cnt);
    }
#endif
    /*
     * Hoist the deleted check: event->deleted is stable under event->mtx.
     * Two separate loops avoid a per-iteration branch on the hot non-deleted path.
     */
    if (event->deleted) {
        TAILQ_FOREACH(entry, &event->any_waiters, link) {
            struct ntsync_q *q = entry->q;
            NTSYNC_DPRINTF("ntsync: try_wake_any_event inspecting entry q=%p index=%u (obj=%p id=%ju)\n",
                   entry->q, entry->index, event, (uintmax_t)event->id);
            mtx_lock(&q->lock);
            if (q->signaled == -1) {
                q->aborted = true;
                q->signaled = entry->index;
                NTSYNC_DPRINTF("ntsync: wake_abort event obj=%p id=%ju -> q=%p (waker tid=%d) signaled=%d aborted=1\n",
                       event, (uintmax_t)event->id, q, curthread->td_tid, q->signaled);
                cv_signal(&q->cv);
            }
            mtx_unlock(&q->lock);
        }
        return;
    }
    TAILQ_FOREACH(entry, &event->any_waiters, link) {
        struct ntsync_q *q = entry->q;
        NTSYNC_DPRINTF("ntsync: try_wake_any_event inspecting entry q=%p index=%u (obj=%p id=%ju)\n",
               entry->q, entry->index, event, (uintmax_t)event->id);
        if (!event->u.event.signaled)
            break;
        mtx_lock(&q->lock);
        if (q->signaled == -1) {
            q->signaled = entry->index;
            if (!event->u.event.manual)
                event->u.event.signaled = false;
            NTSYNC_DPRINTF("ntsync: wake_any event obj=%p id=%ju -> q=%p (waker tid=%d) signaled=%d\n",
                   event, (uintmax_t)event->id, q, curthread->td_tid, q->signaled);
            cv_signal(&q->cv);
        }
        mtx_unlock(&q->lock);
    }
}

static void
ntsync_try_wake_any_obj(struct ntsync_obj *obj)
{
    switch (obj->type) {
    case NTSYNC_TYPE_SEM:
        ntsync_try_wake_any_sem(obj);
        break;
    case NTSYNC_TYPE_MUTEX:
        ntsync_try_wake_any_mutex(obj);
        break;
    case NTSYNC_TYPE_EVENT:
        ntsync_try_wake_any_event(obj);
        break;
    default:
        break;
    }
}

static void
ntsync_try_wake_all(struct ntsync_device *dev, struct ntsync_q *q, struct ntsync_obj *locked_obj)
{
    __u32 i;
    bool can_wake = true;

    mtx_assert(&dev->wait_all_mtx, MA_OWNED);

    for (i = 0; i < q->count; i++) {
        struct ntsync_obj *obj = q->entries[i].obj;
        if (obj == locked_obj)
            continue;
        mtx_lock(&obj->mtx);
    }
    for (i = 0; i < q->count; i++) {
        struct ntsync_obj *obj = q->entries[i].obj;
        /*
         * Treat a deleted object as non-signaled.  The close path marks
         * obj->deleted under wait_all_mtx + obj->mtx (same locks we hold
         * here) and then calls ntsync_try_wake_all_obj, which ends up here.
         * If we did not check deleted we might consume state (decrement a
         * sem count, assign mutex ownership) on an object that is already
         * being freed, leading to use-after-free or incorrect NT semantics.
         * Returning can_wake=false causes the waiter to time out or be
         * woken via ntsync_q_signal_deleted instead.
         */
        if (obj->deleted || !ntsync_is_signaled_owner(obj, q->owner)) {
            can_wake = false;
            break;
        }
    }
    if (can_wake) {
        mtx_lock(&q->lock);
        if (q->signaled == -1) {
            q->signaled = 0;
            for (i = 0; i < q->count; i++) {
                struct ntsync_obj *obj = q->entries[i].obj;
                switch (obj->type) {
                case NTSYNC_TYPE_SEM:
                    obj->u.sem.count--;
                    break;
                case NTSYNC_TYPE_MUTEX:
                    if (obj->u.mutex.ownerdead)
                        q->ownerdead = true;
                    obj->u.mutex.ownerdead = false;
                    obj->u.mutex.count++;
                    obj->u.mutex.owner = (pid_t)q->owner;
                    break;
                case NTSYNC_TYPE_EVENT:
                    if (!obj->u.event.manual)
                        obj->u.event.signaled = false;
                    break;
                default:
                    break;
                }
            }
            cv_signal(&q->cv);
        }
        mtx_unlock(&q->lock);
    }
    for (i = 0; i < q->count; i++) {
        struct ntsync_obj *obj = q->entries[i].obj;
        if (obj == locked_obj)
            continue;
        mtx_unlock(&obj->mtx);
    }
}

static void
ntsync_try_wake_all_obj(struct ntsync_device *dev, struct ntsync_obj *obj)
{
    struct ntsync_q_entry *entry;

    mtx_assert(&dev->wait_all_mtx, MA_OWNED);
    mtx_assert(&obj->mtx, MA_OWNED);

    TAILQ_FOREACH(entry, &obj->all_waiters, link)
        ntsync_try_wake_all(dev, entry->q, obj);
}

/* ---------------------- Scheduling / sleeping ------------------------ */

/*
 * Treat timeout as relative (monotonic) by default.
 * If NTSYNC_WAIT_REALTIME is set, use realtime relative timeout.
 */
static int
ntsync_schedule(struct ntsync_q *q, const struct ntsync_wait_args *args)
{
    int error = 0;

    /*
     * Timeout conversion for Wine/FreeBSD:
     *
     * Wine passes args->timeout as an absolute nanosecond timestamp obtained
     * from clock_gettime(CLOCK_MONOTONIC) (default) or clock_gettime(
     * CLOCK_REALTIME) when NTSYNC_WAIT_REALTIME is set.  This matches the
     * documented Linux ntsync ABI.
     *
     * UINT64_MAX is the "wait forever" sentinel.  Use cv_wait_sig() so the
     * sleep is signal-interruptible (SIGUSR1 for Wine APC delivery, SIGTERM
     * for thread shutdown).  Do NOT use cv_timedwait_sbt with SBT_MAX:
     * cv_timedwait_sbt with flags=0 (relative) computes the absolute deadline
     * as binuptime() + sbt.  SBT_MAX == INT64_MAX, so that addition overflows
     * int64, producing a large negative value that the callout subsystem treats
     * as already-expired.  Every infinite wait would return ETIMEDOUT
     * immediately, breaking all blocking synchronization in Wine.
     *
     * For finite timeouts:
     *   1. Read the appropriate clock: getnanouptime (monotonic, default) or
     *      getnanotime (realtime, NTSYNC_WAIT_REALTIME flag).
     *   2. delta_ns = args->timeout - now_ns.
     *   3. If the deadline has already passed skip the sleep and return
     *      EWOULDBLOCK directly (maps to ETIMEDOUT for the caller).
     *   4. Otherwise compute abs_sbt = sbinuptime() + nstosbt(delta_ns) once
     *      and pass it to cv_timedwait_sig_sbt with C_ABSOLUTE.
     *
     * KNOWN LIMITATION — NTSYNC_WAIT_REALTIME and wall-clock steps:
     *
     * The remaining timeout (delta_ns) is computed from the realtime clock at
     * ioctl entry, but abs_sbt is always anchored to sbinuptime() (the uptime /
     * monotonic clock).  FreeBSD's cv_timedwait_sig_sbt has no realtime-clock
     * variant, so if the system wall clock is stepped during the sleep (NTP
     * correction, settimeofday(2), adjtime(2) large slew, or virtual-machine
     * clock resynchronisation), the timeout will not track the adjustment:
     *
     *   - A step backward shortens the effective wait (timeout fires early).
     *   - A step forward lengthens it (timeout fires late).
     *
     * For Wine's typical use case — process-synchronisation timeouts in the
     * millisecond-to-second range — NTP frequency adjustments (≤500 ppm slew)
     * are negligible.  Large instantaneous steps are rare on well-maintained
     * systems.  Fixing this would require reimplementing the timed sleep on
     * a CLOCK_REALTIME callout, which is disproportionate to the benefit.
     * Document it here so future maintainers do not mistake the behaviour for
     * a bug in the timeout arithmetic.
     */
    NTSYNC_DPRINTF("ntsync: schedule q=%p entering wait timeout=%ju curthread=%p tid=%d q->td=%p\n",
           q, (uintmax_t)args->timeout, curthread, curthread->td_tid, q->td);
    mtx_lock(&q->lock);
    if (q->signaled != -1) {
        NTSYNC_DPRINTF("ntsync: schedule q=%p (tid=%d) already signaled=%d\n",
               q, curthread->td_tid, q->signaled);
    } else if (args->timeout == UINT64_MAX) {
        /*
         * Infinite wait.  Use cv_wait_sig() so that Unix signals (SIGUSR1 for
         * APC delivery, SIGTERM for thread shutdown) can interrupt the sleep.
         * cv_wait_sig returns EINTR when a signal arrives; the loop re-checks
         * q->signaled so it only exits when the object is signaled OR a signal
         * is pending.  EINTR propagates to userspace where Wine's Ntdll layer
         * dispatches the pending APC and then retries the wait.
         */
        NTSYNC_DPRINTF("ntsync: schedule q=%p (tid=%d) infinite wait -> cv_wait_sig\n",
               q, curthread->td_tid);
        while (q->signaled == -1) {
            error = cv_wait_sig(&q->cv, &q->lock);
            if (error != 0)   /* EINTR: signal interrupted the sleep */
                break;
        }
    } else {
        struct timespec now_ts;
        uint64_t abs_ns = args->timeout;
        uint64_t now_ns, timeout_ns;
        sbintime_t abs_sbt;

        if (args->flags & NTSYNC_WAIT_REALTIME)
            getnanotime(&now_ts);
        else
            getnanouptime(&now_ts);

        now_ns = (uint64_t)now_ts.tv_sec * 1000000000ULL +
                 (uint64_t)now_ts.tv_nsec;

        if (abs_ns <= now_ns) {
            /* Deadline already passed — skip sleep entirely. */
            NTSYNC_DPRINTF("ntsync: schedule q=%p (tid=%d) deadline passed -> EWOULDBLOCK\n",
                   q, curthread->td_tid);
            error = EWOULDBLOCK;
        } else {
            timeout_ns = abs_ns - now_ns;
            /*
             * nstosbt() takes int64_t.  timeout_ns is uint64_t and can
             * theoretically exceed INT64_MAX (~9.2e18 ns, ~292 years) when
             * the caller passes args->timeout just below UINT64_MAX (the
             * infinite-wait sentinel).  Without the cap, the implicit
             * truncation to int64_t would produce a negative value; adding it
             * to sbinuptime() would yield a deadline in the past, causing
             * cv_timedwait_sig_sbt to return EWOULDBLOCK immediately —
             * breaking any blocking wait with a near-infinite timeout.
             * Cap to INT64_MAX nanoseconds (~292 years) which is effectively
             * infinite for any real workload.
             */
            if (timeout_ns > (uint64_t)INT64_MAX)
                timeout_ns = (uint64_t)INT64_MAX;
            /*
             * Compute an absolute uptime deadline once and use C_ABSOLUTE so
             * that re-entering the loop after a spurious wakeup re-arms to the
             * same fixed point in time rather than the original full duration.
             */
            abs_sbt = sbinuptime() + nstosbt((int64_t)timeout_ns);
            while (q->signaled == -1 && error == 0) {
                NTSYNC_DPRINTF("ntsync: schedule q=%p (tid=%d) cv_timedwait_sbt abs_sbt=%jx\n",
                       q, curthread->td_tid, (uintmax_t)abs_sbt);
                error = cv_timedwait_sig_sbt(&q->cv, &q->lock, abs_sbt, 0, C_ABSOLUTE);
                NTSYNC_DPRINTF("ntsync: schedule q=%p (tid=%d) woke error=%d signaled=%d aborted=%d\n",
                       q, curthread->td_tid, error, q->signaled, q->aborted);
                if (error == EWOULDBLOCK)
                    break;
            }
        }
    }
    /* capture aborted state before unlocking */
    bool aborted = q->aborted;
    mtx_unlock(&q->lock);

    if (q->signaled != -1) {
        if (aborted)
            return (ETIMEDOUT);
        return (0);
    }
    if (error == EWOULDBLOCK)
        return (ETIMEDOUT);
    return (error);
}

/* ---------------------- setup_wait ---------------------------------- */

static int
ntsync_setup_wait(struct ntsync_device *dev, const struct ntsync_wait_args *args, bool all, struct ntsync_q **out_q)
{
    int fds[NTSYNC_MAX_WAIT_COUNT + 1];
    __u32 count = args->count;
    __u32 total_count;
    struct ntsync_q *q;
    __u32 i, j;
    size_t size;
    int error = 0;

    if (args->pad || (args->flags & ~NTSYNC_WAIT_REALTIME))
        return (EINVAL);
    if (count > NTSYNC_MAX_WAIT_COUNT)
        return (EINVAL);

    /* Copy exactly 'count' elements from the user-supplied objs array.
     * If args->alert is set, append it separately. Previously the code
     * increased the copyin size and then appended args->alert, which
     * overwrote the last copied element.
     */
    size = count * sizeof(fds[0]);

    /* Defensive check: ensure we won't overflow the local fds[] buffer. */
    if (size > sizeof(fds)) {
        NTSYNC_DPRINTF("ntsync: setup_wait: requested objs size %zu exceeds fds buffer %zu\n",
               size, sizeof(fds));
        return (EINVAL);
    }

    /*
     * Resolve the objs pointer once through ntsync_objs_uptr() so that
     * all diagnostic prints and the actual copyin() use the same masked
     * value.  For ILP32 (WOW64) callers the upper 32 bits are cleared;
     * for 64-bit callers the raw pointer is used unchanged.
     */
    void *objs_ptr = ntsync_objs_uptr(curthread, args);

    /* Diagnostic: log planned copy of objs array from userland */
    NTSYNC_DPRINTF("ntsync: setup_wait called args=%p count=%u alert=%u total_bytes=%zu user_objs=%p\n",
           args, count, args->alert, size, objs_ptr);

    if (size > 0) {
        NTSYNC_DPRINTF("ntsync: setup_wait about to copy objs user_ptr=%p bytes=%zu\n",
               objs_ptr, size);
        if (ntsync_safe_copyin(objs_ptr, fds, size) != 0) {
            NTSYNC_DPRINTF("ntsync: setup_wait copyin(objs) failed user_ptr=%p bytes=%zu\n",
                   objs_ptr, size);
            return (EFAULT);
        }
        NTSYNC_DPRINTF("ntsync: setup_wait copyin(objs) succeeded count=%u bytes=%zu\n",
               count, size);
    }

    total_count = count;
    if (args->alert) {
        /* append the explicit alert value (not part of the user array) */
        fds[total_count++] = args->alert;
    }

    /* Diagnostic: log allocation size for q and total_count */
    NTSYNC_DPRINTF("ntsync: setup_wait allocating q entries total_count=%u entry_bytes=%zu q_size=%zu\n",
           total_count, total_count * sizeof(struct ntsync_q_entry),
           sizeof(*q) + total_count * sizeof(struct ntsync_q_entry));
    q = uma_zalloc(ntsync_q_zone, M_WAITOK);
    counter_u64_add(ntsync_active_q_count, 1);

    /*
     * Pass total_count so ntsync_q_init zeroes only the slots that will be
     * used, instead of the full 65-slot array.
     */
    ntsync_q_init(q, args->owner, all, count, total_count);

    for (i = 0; i < total_count; i++) {
        struct ntsync_q_entry *entry = &q->entries[i];
        struct ntsync_obj *obj;

        error = ntsync_get_obj(curthread, fds[i], dev, &obj);
        if (error != 0)
            goto err;

        /*
         * Defensive: immediately check whether the object is already
         * being deleted. If so, drop the reference and fail setup.
         * For wait_any (all == false) we also insert the entry into
         * the object's any_waiters list here while holding obj->mtx
         * and increment active_waiters. This guarantees the object
         * cannot be freed and reallocated at the same address
         * between setup and the later sleep call.
         *
         * For wait_all (all == true) we do not insert here because
         * wait_all must acquire dev->wait_all_mtx and follow the
         * established lock ordering (dev->wait_all_mtx -> obj->mtx).
         */
        mtx_lock(&obj->mtx);
        if (obj->deleted) {
            mtx_unlock(&obj->mtx);
            ntsync_obj_drop(obj, "setup_wait:deleted_check");
            error = EINVAL;
            goto err;
        }

       /* Fill the entry fields first, then insert under obj->mtx.
        * This prevents exposing a partially-initialized list node
        * to wake helpers (which caused the hang you observed).
        */
       entry->obj = obj;
       entry->q = q;
       entry->index = i;

       if (!all) {
           TAILQ_INSERT_TAIL(&obj->any_waiters, entry, link);
           obj->active_waiters++;

           NTSYNC_DPRINTF("ntsync: setup_wait inserted any_waiters entry idx=%u obj=%p id=%ju active_waiters=%d deleted=%d\n",
                  i, obj, (uintmax_t)obj->id, obj->active_waiters, obj->deleted);

           /*
            * Attempt an immediate wake while obj->mtx is still held —
            * eliminates the separate post-setup lock/unlock pass in
            * wait_any (saving one mtx_lock+unlock pair per object).
            * Safe because entry is fully initialised and inserted;
            * if the object is already signaled, q->signaled is set here
            * and ntsync_schedule returns without sleeping.
            */
           ntsync_try_wake_any_obj(obj);
       }
       mtx_unlock(&obj->mtx);

        /*
         * Duplicate object check for WAIT_ALL only.
         *
         * WAIT_ALL requires all listed objects to be acquired atomically in a
         * single operation.  Listing the same object fd twice is semantically
         * incoherent (you cannot atomically acquire X and X simultaneously in
         * a way that is distinguishable from acquiring X once) and would
         * deadlock ntsync_try_wake_all, which locks every object in the list
         * in array order — a duplicate would attempt to lock the same FreeBSD
         * mutex twice, triggering a panic under WITNESS/INVARIANTS.
         *
         * WAIT_ANY does NOT have this restriction.  The Linux ntsync ABI
         * explicitly permits duplicate fds in WAIT_ANY: the first matching
         * entry wins and consumes exactly one token.  This is tested by the
         * Linux kernel ntsync test suite (see ntsync.cpp, test_wait_any).
         * The alert slot (i >= count) is also excluded because Wine
         * legitimately passes the same event fd as both a wait target and
         * the alert.
         */
        if (all && i < count) {
            for (j = 0; j < i; j++) {
                if (obj == q->entries[j].obj) {
                    ntsync_obj_drop(obj, "setup_wait:dup_obj");
                    error = EINVAL;
                    goto err;
                }
            }
        }

        /* entry already initialized above for wait_any; for wait_all the
         * caller will insert under dev->wait_all_mtx as before.
         */
    }

    *out_q = q;
    return (0);

err:
    for (j = 0; j < i; j++) {
        struct ntsync_q_entry *entry = &q->entries[j];
        struct ntsync_obj *obj = entry->obj;
        /*
         * For wait_any, entries were inserted into obj->any_waiters and
         * active_waiters was incremented.  Undo that before dropping the
         * reference, otherwise the close path will drain forever and the
         * entry pointer becomes dangling in the waiter list.
         */
        if (!all && entry->q == q) {
            mtx_lock(&obj->mtx);
            TAILQ_REMOVE(&obj->any_waiters, entry, link);
            entry->q = NULL;
            if (obj->active_waiters > 0)
                obj->active_waiters--;
            if (obj->active_waiters == 0)
                cv_signal(&obj->waiters_cv);
            mtx_unlock(&obj->mtx);
        }
        ntsync_obj_drop(obj, "setup_wait:cleanup");
    }
    counter_u64_add(ntsync_active_q_count, -1);
    uma_zfree(ntsync_q_zone, q);
    return (error);
}

/* ---------------------- Diagnostic helper ---------------------------- */

/*
 * ntsync_diag_dump_wait_args()
 *
 * Print a hex dump of a ntsync_wait_args struct after copyin, together with
 * the key field offsets/sizes.  Shared by wait_any and wait_all to avoid
 * maintaining two diverging copies of the same diagnostic block.
 *
 * Only active when NTSYNC_DEBUG is defined; expands to nothing in production.
 */
#ifdef NTSYNC_DEBUG
static void
ntsync_diag_dump_wait_args(const char *caller, const struct ntsync_wait_args *args)
{
    size_t off_objs = offsetof(struct ntsync_wait_args, objs);
    size_t sz_args  = sizeof(struct ntsync_wait_args);
    size_t sz_objs  = sizeof(args->objs);
    const unsigned char *p = (const unsigned char *)args;
    size_t n = sz_args;
    size_t i;

    NTSYNC_DPRINTF("ntsync: %s DIAG sizeof(wait_args)=%zu offsetof(objs)=%zu "
           "sizeof(objs)=%zu sizeof(void*)=%zu\n",
           caller, sz_args, off_objs, sz_objs, sizeof(void *));
    NTSYNC_DPRINTF("ntsync: %s safe_copyin succeeded; "
           "args.count=%u args.alert=%u args.objs=%ju\n",
           caller, args->count, args->alert, (uintmax_t)args->objs);

    if (n > 64) n = 64;
    NTSYNC_DPRINTF("ntsync: %s DIAG raw args first %zu bytes (hex):\n", caller, n);
    for (i = 0; i < n; i += 8) {
        unsigned char b0 = (i + 0 < n) ? p[i + 0] : 0;
        unsigned char b1 = (i + 1 < n) ? p[i + 1] : 0;
        unsigned char b2 = (i + 2 < n) ? p[i + 2] : 0;
        unsigned char b3 = (i + 3 < n) ? p[i + 3] : 0;
        unsigned char b4 = (i + 4 < n) ? p[i + 4] : 0;
        unsigned char b5 = (i + 5 < n) ? p[i + 5] : 0;
        unsigned char b6 = (i + 6 < n) ? p[i + 6] : 0;
        unsigned char b7 = (i + 7 < n) ? p[i + 7] : 0;
        NTSYNC_DPRINTF("ntsync: DIAG  %02x %02x %02x %02x %02x %02x %02x %02x\n",
               b0, b1, b2, b3, b4, b5, b6, b7);
    }
    /* Print the 8 raw bytes at the objs offset for quick cross-check */
    if (off_objs + sz_objs <= sizeof(*args)) {
        const uint8_t *q = (const uint8_t *)args + off_objs;
        NTSYNC_DPRINTF("ntsync: %s DIAG objs raw bytes at offset %zu: "
               "%02x%02x%02x%02x%02x%02x%02x%02x\n",
               caller, off_objs,
               q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7]);
    }
}
#else
#define ntsync_diag_dump_wait_args(caller, args) ((void)0)
#endif /* NTSYNC_DEBUG */

/* ---------------------- WAIT_ANY ------------------------------------ */

static int
ntsync_wait_any(struct ntsync_device *dev, void *uarg, void *udata)
{
    struct ntsync_wait_args args;
    struct ntsync_wait_args *user_args = uarg;
    struct ntsync_q *q;
    __u32 total_count;
    __u32 i;
    int error;

    NTSYNC_DPRINTF("ntsync: wait_any called user_args=%p expected_size=%zu\n",
           user_args, sizeof(args));
    if (ntsync_kern_copyin(user_args, &args, sizeof(args)) != 0) {
        NTSYNC_DPRINTF("ntsync: wait_any kern_copyin failed user_ptr=%p expected_size=%zu\n",
               user_args, sizeof(args));
        return (EFAULT);
    }
    ntsync_diag_dump_wait_args("wait_any", &args);

    error = ntsync_setup_wait(dev, &args, false, &q);
    if (error != 0)
        return (error);

    total_count = args.count;
    if (args.alert)
        total_count++;

    error = ntsync_schedule(q, &args);

    /* Unregister entries and decrement active_waiters under obj->mtx.
     * Also drop the waiter-held reference we took at registration time.
     */
    for (i = 0; i < total_count; i++) {
        struct ntsync_q_entry *entry = &q->entries[i];
        struct ntsync_obj *obj = entry->obj;

        mtx_lock(&obj->mtx);
        if (entry->q == q) {
            TAILQ_REMOVE(&obj->any_waiters, entry, link);
            entry->q = NULL;
            /* decrement active waiter count and signal close path if zero */
            if (obj->active_waiters > 0)
                obj->active_waiters--;
            else
                printf("ntsync: warning: active_waiters already zero for obj=%p id=%ju\n",
                       obj, (uintmax_t)obj->id);
            if (obj->active_waiters == 0)
                cv_signal(&obj->waiters_cv);
            mtx_unlock(&obj->mtx);
            /* Drop the reference we acquired when registering the waiter. */
            ntsync_obj_drop(obj, "wait_any:unregister");
        } else {
            /*
             * This branch is believed unreachable: entry->q is only
             * cleared by this teardown loop itself.  ntsync_q_signal_deleted
             * (called by obj_close) signals q->cv but does NOT
             * clear entry->q and does NOT remove entries from any_waiters.
             *
             * Decrement active_waiters so the obj_close drain loop is not
             * blocked on a counter that would otherwise never reach zero.
             * Signal waiters_cv in case obj_close is already sleeping on it.
             * KASSERT fires in INVARIANTS kernels to catch the regression.
             */
            if (obj->active_waiters > 0)
                obj->active_waiters--;
            if (obj->active_waiters == 0)
                cv_signal(&obj->waiters_cv);
            mtx_unlock(&obj->mtx);
            KASSERT(0, ("ntsync wait_any teardown: entry->q != q, "
                "dropping ref defensively to unblock obj_close drain"));
            ntsync_obj_drop(obj, "wait_any:unregister_orphan");
        }
    }

    if (q->signaled != -1) {
        __u32 index = q->signaled;
        int ret;
        if (q->aborted)
            ret = ETIMEDOUT;
        else
            ret = q->ownerdead ? EOWNERDEAD : 0;

    /*
     * Update the index field in two places:
     *
     * 1. The kernel buffer (uarg): kern_ioctl copies this back to user space
     *    automatically when the ioctl returns 0.  Without this write the
     *    buffer retains the stale value from the original copyin, which
     *    kern_ioctl would copy back on success, overwriting any direct
     *    copyout we did.
     *
     * 2. Direct copyout to udata (original user pointer from td->td_sa.args[2]):
     *    kern_ioctl discards the kernel buffer on non-zero returns (EOWNERDEAD) or ETIMEDOUT (aborted),
     *    so we bypass it for that case only.  Guarding with ret != 0 avoids a
     *    redundant second write on the success path, consistent with the same
     *    fix applied to ntsync_mutex_read.
     */
    ((struct ntsync_wait_args *)uarg)->index = index;
    if (ret != 0) {
        if (copyout(&index,
                    (uint8_t *)udata + offsetof(struct ntsync_wait_args, index),
                    sizeof(index)) != 0)
            ret = EFAULT;
    }
    NTSYNC_DPRINTF("ntsync: wait_any copyout index=%u to udata=%p ret=%d\n",
           index, udata, ret);

        NTSYNC_DPRINTF("ntsync: wait_any returning to user ret=%d signaled=%u aborted=%d ownerdead=%d\n",
               ret, index, q->aborted, q->ownerdead);
        counter_u64_add(ntsync_active_q_count, -1);
        uma_zfree(ntsync_q_zone, q);
        return (ret);
    }

    if (error == 0)
        error = ETIMEDOUT;

    counter_u64_add(ntsync_active_q_count, -1);
    uma_zfree(ntsync_q_zone, q);
    return (error);
}

/* ---------------------- WAIT_ALL ------------------------------------ */

static int
ntsync_wait_all(struct ntsync_device *dev, void *uarg, void *udata)
{
    struct ntsync_wait_args args;
    struct ntsync_wait_args *user_args = uarg;
    struct ntsync_q *q;
    __u32 i, j;
    int error;

    NTSYNC_DPRINTF("ntsync: wait_all called user_args=%p expected_size=%zu\n",
           user_args, sizeof(args));
    if (ntsync_kern_copyin(user_args, &args, sizeof(args)) != 0) {
        NTSYNC_DPRINTF("ntsync: wait_all kern_copyin failed user_ptr=%p expected_size=%zu\n",
               user_args, sizeof(args));
        return (EFAULT);
    }
    ntsync_diag_dump_wait_args("wait_all", &args);

    if (args.count == 0)
        return (EINVAL);

    error = ntsync_setup_wait(dev, &args, true, &q);
    if (error != 0)
        return (error);

    mtx_lock(&dev->wait_all_mtx);
    for (i = 0; i < args.count; i++) {
        struct ntsync_q_entry *entry = &q->entries[i];
        struct ntsync_obj *obj = entry->obj;
        mtx_lock(&obj->mtx);
        if (obj->deleted) {
            /*
             * Object was closed between setup_wait and insertion.
             * wait_all_mtx is held; jump to the error path to unwind
             * the entries already inserted and drop all remaining refs.
             */
            mtx_unlock(&obj->mtx);
            error = EINVAL;
            goto wait_all_insert_err;
        }
        TAILQ_INSERT_TAIL(&obj->all_waiters, entry, link);
        obj->all_hint++;
        obj->active_waiters++;
        NTSYNC_DPRINTF("ntsync: wait_all registered all_waiters entry idx=%u obj=%p id=%ju "
               "all_hint=%d active_waiters=%d\n",
               i, obj, (uintmax_t)obj->id, obj->all_hint, obj->active_waiters);
        mtx_unlock(&obj->mtx);
    }
    ntsync_try_wake_all(dev, q, NULL);
    mtx_unlock(&dev->wait_all_mtx);

    if (args.alert) {
        struct ntsync_q_entry *entry = &q->entries[args.count];
        struct ntsync_obj *obj = entry->obj;
        mtx_lock(&obj->mtx);
        if (obj->deleted) {
            /*
             * Alert object was closed in the window between setup_wait
             * and here.  Its ref is still held; jump to the error path
             * which drops it and unwinds the main all_waiters entries.
             */
            mtx_unlock(&obj->mtx);
            error = EINVAL;
            goto wait_all_alert_err;
        }
        TAILQ_INSERT_TAIL(&obj->any_waiters, entry, link);
        obj->active_waiters++;
        NTSYNC_DPRINTF("ntsync: wait_all registering alert on obj %p id=%ju (deleted=%d active_waiters=%d)\n",
               obj, (uintmax_t)obj->id, obj->deleted, obj->active_waiters);
        ntsync_try_wake_any_obj(obj);
        mtx_unlock(&obj->mtx);
    }

    error = ntsync_schedule(q, &args);

    mtx_lock(&dev->wait_all_mtx);
    for (i = 0; i < args.count; i++) {
        struct ntsync_q_entry *entry = &q->entries[i];
        struct ntsync_obj *obj = entry->obj;
        mtx_lock(&obj->mtx);

        if (entry->q == q) {
            TAILQ_REMOVE(&obj->all_waiters, entry, link);
            entry->q = NULL;
            obj->all_hint--;
            if (obj->active_waiters > 0)
                obj->active_waiters--;
            else
                printf("ntsync: warning: active_waiters already zero for obj=%p id=%ju\n",
                       obj, (uintmax_t)obj->id);
            if (obj->active_waiters == 0)
                cv_signal(&obj->waiters_cv);
            mtx_unlock(&obj->mtx);
        } else {
            /*
             * Defensive branch: entry->q != q means something else already
             * removed this entry from obj->all_waiters and cleared entry->q.
             * This is not expected to occur — entry->q is only ever cleared by
             * this teardown loop itself — but handle it safely: adjust all_hint,
             * decrement active_waiters so obj_close's drain loop can complete,
             * and drop the ntsync_get_obj reference that was acquired for this
             * entry in setup_wait.  Without the drop the object would leak a
             * refcount and never be freed.
             */
            obj->all_hint--;
            if (obj->active_waiters > 0)
                obj->active_waiters--;
            if (obj->active_waiters == 0)
                cv_signal(&obj->waiters_cv);
            mtx_unlock(&obj->mtx);
        }
    }
    /*
     * Release wait_all_mtx BEFORE calling ntsync_obj_drop.  ntsync_obj_unref
     * acquires wait_all_mtx when the refcount hits zero; if we still held it
     * here a concurrent close() that races the final drop would self-deadlock.
     */
    mtx_unlock(&dev->wait_all_mtx);
    for (i = 0; i < args.count; i++)
        ntsync_obj_drop(q->entries[i].obj, "wait_all:unregister");

    if (args.alert) {
        struct ntsync_q_entry *entry = &q->entries[args.count];
        struct ntsync_obj *obj = entry->obj;
        mtx_lock(&obj->mtx);
        if (entry->q == q) {
            TAILQ_REMOVE(&obj->any_waiters, entry, link);
            entry->q = NULL;
            /* decrement active waiter count and signal close path if zero */
            if (obj->active_waiters > 0)
                obj->active_waiters--;
            if (obj->active_waiters == 0)
                cv_signal(&obj->waiters_cv);
        } else {
            /*
             * Defensive branch: entry->q != q is not expected to occur —
             * entry->q is only cleared by this teardown.  Handle it safely
             * for symmetry with all other teardown sites: decrement
             * active_waiters so obj_close's drain loop is not blocked on
             * a counter that would never reach zero.
             */
            if (obj->active_waiters > 0)
                obj->active_waiters--;
            if (obj->active_waiters == 0)
                cv_signal(&obj->waiters_cv);
        }
        mtx_unlock(&obj->mtx);
        ntsync_obj_drop(obj, "wait_all:alert_unregister");
    }

    if (q->signaled != -1) {
        __u32 index = q->signaled;
        int ret;
        if (q->aborted)
            ret = ETIMEDOUT;
        else
            ret = q->ownerdead ? EOWNERDEAD : 0;

    /*
     * Same dual-write as wait_any: update the kernel buffer for the success
     * path (kern_ioctl copies it back on ret==0) AND copyout directly to
     * udata for the EOWNERDEAD path (kern_ioctl discards kernel buffer on
     * non-zero return).  Guard the copyout with ret != 0 to avoid a
     * redundant second write on the success path.
     */
    ((struct ntsync_wait_args *)uarg)->index = index;
    if (ret != 0) {
        if (copyout(&index,
                    (uint8_t *)udata + offsetof(struct ntsync_wait_args, index),
                    sizeof(index)) != 0)
            ret = EFAULT;
    }
    NTSYNC_DPRINTF("ntsync: wait_all copyout index=%u to udata=%p ret=%d\n",
           index, udata, ret);

        NTSYNC_DPRINTF("ntsync: wait_all returning to user ret=%d signaled=%u aborted=%d ownerdead=%d\n",
               ret, index, q->aborted, q->ownerdead);
        counter_u64_add(ntsync_active_q_count, -1);
        uma_zfree(ntsync_q_zone, q);
        return (ret);
    }

    if (error == 0)
        error = ETIMEDOUT;

    counter_u64_add(ntsync_active_q_count, -1);
    uma_zfree(ntsync_q_zone, q);
    return (error);

wait_all_insert_err:
    /*
     * A deleted object was detected at entries[i] during all_waiters
     * insertion.  wait_all_mtx is still held.  Unwind the j < i entries
     * already inserted, release the mutex, then drop all refs
     * (entries[0..i-1] that were inserted, plus entries[i..total-1] that
     * were never inserted).
     *
     * Drop all refs AFTER releasing wait_all_mtx: ntsync_obj_unref
     * acquires wait_all_mtx when refcnt hits zero, so dropping while
     * holding it would self-deadlock on a concurrent final close().
     */
    for (j = 0; j < i; j++) {
        struct ntsync_q_entry *e = &q->entries[j];
        struct ntsync_obj *o = e->obj;
        mtx_lock(&o->mtx);
        TAILQ_REMOVE(&o->all_waiters, e, link);
        e->q = NULL;
        o->all_hint--;
        if (o->active_waiters > 0)
            o->active_waiters--;
        if (o->active_waiters == 0)
            cv_signal(&o->waiters_cv);
        mtx_unlock(&o->mtx);
    }
    mtx_unlock(&dev->wait_all_mtx);
    {
        __u32 total = args.count + (args.alert ? 1 : 0);
        for (j = 0; j < i; j++)
            ntsync_obj_drop(q->entries[j].obj, "wait_all:insert_err_unwind");
        for (j = i; j < total; j++)
            ntsync_obj_drop(q->entries[j].obj, "wait_all:insert_err_cleanup");
    }
    counter_u64_add(ntsync_active_q_count, -1);
    uma_zfree(ntsync_q_zone, q);
    return (error);

wait_all_alert_err:
    /*
     * The alert object was closed before insertion into any_waiters.
     * The main all_waiters entries (0..count-1) were all successfully
     * inserted.  Unwind the main entries under wait_all_mtx, then drop
     * all refs AFTER releasing the mutex to avoid a self-deadlock in
     * ntsync_obj_unref (which also acquires wait_all_mtx at final drop).
     */
    ntsync_obj_drop(q->entries[args.count].obj,
        "wait_all:alert_deleted_cleanup");
    mtx_lock(&dev->wait_all_mtx);
    for (i = 0; i < args.count; i++) {
        struct ntsync_q_entry *e = &q->entries[i];
        struct ntsync_obj *o = e->obj;
        mtx_lock(&o->mtx);
        if (e->q == q) {
            TAILQ_REMOVE(&o->all_waiters, e, link);
            e->q = NULL;
            o->all_hint--;
            if (o->active_waiters > 0)
                o->active_waiters--;
            if (o->active_waiters == 0)
                cv_signal(&o->waiters_cv);
            mtx_unlock(&o->mtx);
        } else {
            o->all_hint--;
            if (o->active_waiters > 0)
                o->active_waiters--;
            if (o->active_waiters == 0)
                cv_signal(&o->waiters_cv);
            mtx_unlock(&o->mtx);
        }
    }
    mtx_unlock(&dev->wait_all_mtx);
    for (i = 0; i < args.count; i++)
        ntsync_obj_drop(q->entries[i].obj, "wait_all:alert_err_unwind");
    counter_u64_add(ntsync_active_q_count, -1);
    uma_zfree(ntsync_q_zone, q);
    return (error);
}

/* ---------------------- Object ioctls -------------------------------- */

static int
ntsync_sem_release(struct ntsync_obj *sem, void *uarg)
{
    __u32 count;
    __u32 prev;
    int error;

    if (sem->type != NTSYNC_TYPE_SEM)
        return (EINVAL);

    if (ntsync_kern_copyin(uarg, &count, sizeof(count)) != 0)
        return (EFAULT);

    struct ntsync_device *dev = sem->dev;
    bool need_all;

    /*
     * Fast path: acquire only sem->mtx first and inspect all_hint.
     * all_hint is modified exclusively under wait_all_mtx + obj->mtx,
     * so seeing all_hint == 0 while holding obj->mtx guarantees no
     * concurrent WAIT_ALL insertion can observe the state change we are
     * about to make — a new WAIT_ALL waiter would need obj->mtx (which
     * we hold) before it could insert.  Only escalate to the global
     * wait_all_mtx when at least one WAIT_ALL waiter is registered.
     */
    mtx_lock(&sem->mtx);
    need_all = (dev != NULL && sem->all_hint > 0);
    if (need_all) {
        /* Slow path: re-acquire in correct order (wait_all_mtx → obj->mtx). */
        mtx_unlock(&sem->mtx);
        mtx_lock(&dev->wait_all_mtx);
        mtx_lock(&sem->mtx);
    }
    prev = sem->u.sem.count;
    error = ntsync_sem_release_state(sem, count);
    if (error == 0) {
        if (need_all)
            ntsync_try_wake_all_obj(dev, sem);
        ntsync_try_wake_any_sem(sem);
        ntsync_obj_notify(sem);
    }
    mtx_unlock(&sem->mtx);
    if (need_all)
        mtx_unlock(&dev->wait_all_mtx);

    if (error == 0) {
        if (ntsync_kern_copyout(&prev, uarg, sizeof(prev)) != 0)
            error = EFAULT;
    }
    return (error);
}

static int
ntsync_sem_read(struct ntsync_obj *sem, void *uarg)
{
    struct ntsync_sem_args args;

    if (sem->type != NTSYNC_TYPE_SEM)
        return (EINVAL);

    mtx_lock(&sem->mtx);
    args.count = sem->u.sem.count;
    args.max = sem->u.sem.max;
    mtx_unlock(&sem->mtx);

    if (ntsync_kern_copyout(&args, uarg, sizeof(args)) != 0)
        return (EFAULT);
    return (0);
}

static int
ntsync_mutex_unlock(struct ntsync_obj *mutex, void *uarg)
{
    struct ntsync_mutex_args args;
    __u32 prev;
    int error;

    if (mutex->type != NTSYNC_TYPE_MUTEX)
        return (EINVAL);

    NTSYNC_DPRINTF("ntsync: MUTEX_UNLOCK called uarg=%p obj=%p id=%ju type=%d owner=%u count=%u\n",
        uarg, mutex, (uintmax_t)mutex->id, mutex->type,
        mutex->u.mutex.owner, mutex->u.mutex.count);

    if (ntsync_kern_copyin(uarg, &args, sizeof(args)) != 0)
        return (EFAULT);

    NTSYNC_DPRINTF("ntsync: MUTEX_UNLOCK args.owner=%u args.count=%u\n",
        args.owner, args.count);

    if (!args.owner)
        return (EINVAL);

    struct ntsync_device *dev = mutex->dev;
    bool need_all;

    mtx_lock(&mutex->mtx);
    need_all = (dev != NULL && mutex->all_hint > 0);
    if (need_all) {
        mtx_unlock(&mutex->mtx);
        mtx_lock(&dev->wait_all_mtx);
        mtx_lock(&mutex->mtx);
    }
    prev = mutex->u.mutex.count;
    error = ntsync_mutex_unlock_state(mutex, &args);
    if (error == 0) {
        if (need_all)
            ntsync_try_wake_all_obj(dev, mutex);
        ntsync_try_wake_any_mutex(mutex);
        ntsync_obj_notify(mutex);
    }
    mtx_unlock(&mutex->mtx);
    if (need_all)
        mtx_unlock(&dev->wait_all_mtx);

    if (error == 0) {
        /* Copy the single count field back to user space using helper. */
        if (ntsync_copyout_field(uarg, offsetof(struct ntsync_mutex_args, count),
                                 &prev, sizeof(prev)) != 0)
            error = EFAULT;
    }
    return (error);
}

static int
ntsync_mutex_kill(struct ntsync_obj *mutex, void *uarg)
{
    __u32 owner;
    int error;

    if (mutex->type != NTSYNC_TYPE_MUTEX)
        return (EINVAL);

    if (ntsync_kern_copyin(uarg, &owner, sizeof(owner)) != 0)
        return (EFAULT);
    if (!owner)
        return (EINVAL);

    struct ntsync_device *dev = mutex->dev;
    bool need_all;

    mtx_lock(&mutex->mtx);
    need_all = (dev != NULL && mutex->all_hint > 0);
    if (need_all) {
        mtx_unlock(&mutex->mtx);
        mtx_lock(&dev->wait_all_mtx);
        mtx_lock(&mutex->mtx);
    }
    error = ntsync_mutex_kill_state(mutex, owner);
    if (error == 0) {
        if (need_all)
            ntsync_try_wake_all_obj(dev, mutex);
        ntsync_try_wake_any_mutex(mutex);
        ntsync_obj_notify(mutex);
    }
    mtx_unlock(&mutex->mtx);
    if (need_all)
        mtx_unlock(&dev->wait_all_mtx);
    return (error);
}

static int
ntsync_mutex_read(struct ntsync_obj *mutex, void *uarg, void *udata)
{
    struct ntsync_mutex_args args;
    int error = 0;

    if (mutex->type != NTSYNC_TYPE_MUTEX)
        return (EINVAL);

    mtx_lock(&mutex->mtx);
    args.count = mutex->u.mutex.count;
    args.owner = (mutex->u.mutex.ownerdead ? 0 : mutex->u.mutex.owner);
    if (mutex->u.mutex.ownerdead)
        error = EOWNERDEAD;
    mtx_unlock(&mutex->mtx);

    /*
     * On the EOWNERDEAD path kern_ioctl discards the kernel buffer, so we
     * must copyout directly to the original user pointer.  On the success
     * path kern_ioctl copies uarg back automatically; the copyout is only
     * needed for the non-zero return.  Guard it accordingly to avoid a
     * redundant second write to userspace on every successful MUTEX_READ.
     */
    memcpy(uarg, &args, sizeof(args));
    if (error != 0) {
        if (copyout(&args, udata, sizeof(args)) != 0)
            return (EFAULT);
    }
    return (error);
}

static int
ntsync_event_set_reset(struct ntsync_obj *event, void *uarg, bool set, bool pulse)
{
    __u32 prev;
    int error = 0;

    if (event->type != NTSYNC_TYPE_EVENT)
        return (EINVAL);

    struct ntsync_device *dev = event->dev;
    bool need_all;

    mtx_lock(&event->mtx);
    need_all = (dev != NULL && event->all_hint > 0);
    if (need_all) {
        mtx_unlock(&event->mtx);
        mtx_lock(&dev->wait_all_mtx);
        mtx_lock(&event->mtx);
    }
    prev = event->u.event.signaled;
    event->u.event.signaled = set;
    if (set) {
        if (need_all)
            ntsync_try_wake_all_obj(dev, event);
        ntsync_try_wake_any_event(event);
        ntsync_obj_notify(event);
        if (pulse)
            event->u.event.signaled = false;
    }
    mtx_unlock(&event->mtx);
    if (need_all)
        mtx_unlock(&dev->wait_all_mtx);

    if (ntsync_kern_copyout(&prev, uarg, sizeof(prev)) != 0)
        error = EFAULT;
    return (error);
}

static int
ntsync_event_read(struct ntsync_obj *event, void *uarg)
{
    struct ntsync_event_args args;

    if (event->type != NTSYNC_TYPE_EVENT)
        return (EINVAL);

    mtx_lock(&event->mtx);
    args.manual = event->u.event.manual;
    args.signaled = event->u.event.signaled;
    mtx_unlock(&event->mtx);

    if (ntsync_kern_copyout(&args, uarg, sizeof(args)) != 0)
        return (EFAULT);
    return (0);
}

/* ---------------------- Control create_* helpers --------------------- */

/* These return 0 on success and place the new fd into *fdp. */

static int
ntsync_create_sem(struct ntsync_device *dev, void *uarg, int *fdp)
{
    struct ntsync_sem_args args;
    struct ntsync_obj *sem;
    int fd, error;

    /* Debug: log the user pointer and expected size before copyin */
    NTSYNC_DPRINTF("ntsync: CREATE_SEM ioctl: user_ptr=%p expected_len=%zu curthread=%p pid=%d\n",
           uarg, sizeof(args), (void *)curthread, curthread ? curthread->td_proc->p_pid : -1);

    if (ntsync_kern_copyin(uarg, &args, sizeof(args)) != 0) {
        NTSYNC_DPRINTF("ntsync: CREATE_SEM kern_copyin failed user_ptr=%p len=%zu\n",
               uarg, sizeof(args));
        return (EFAULT);
    }

    /* Debug: show the contents the kernel received */
    NTSYNC_DPRINTF("ntsync: CREATE_SEM copyin succeeded; args.count=%u args.max=%u\n",
           args.count, args.max);

    if (args.count > args.max)
        return (EINVAL);

    sem = ntsync_alloc_obj(dev, NTSYNC_TYPE_SEM);
    sem->u.sem.count = args.count;
    sem->u.sem.max = args.max;
    error = ntsync_obj_get_fd(curthread, sem, &fd);
    ntsync_obj_drop(sem, "create_sem:post_get_fd");
    if (error != 0)
        return (error);
    *fdp = fd;
    return (0);
}

static int
ntsync_create_mutex(struct ntsync_device *dev, void *uarg, int *fdp)
{
    struct ntsync_mutex_args args;
    struct ntsync_obj *mutex;
    int fd, error;

    if (ntsync_kern_copyin(uarg, &args, sizeof(args)) != 0)
        return (EFAULT);
    if (!!args.owner != !!args.count)
        return (EINVAL);
    /*
     * count == UINT32_MAX is a valid initial state meaning the mutex is at
     * maximum recursion depth.  ntsync_try_wake_any_mutex treats count ==
     * UINT_MAX as a sentinel that prevents new waiters from acquiring the
     * mutex (WAIT_ANY returns ETIMEDOUT), which is the correct ABI behaviour
     * for a saturated mutex.  Do NOT reject UINT32_MAX here — the Linux
     * ntsync test suite explicitly creates a mutex in this state and expects
     * creation to succeed followed by ETIMEDOUT on any WAIT_ANY attempt.
     */

    mutex = ntsync_alloc_obj(dev, NTSYNC_TYPE_MUTEX);
    mutex->u.mutex.count = args.count;
    mutex->u.mutex.owner = (pid_t)args.owner;
    mutex->u.mutex.ownerdead = false;
    error = ntsync_obj_get_fd(curthread, mutex, &fd);
    ntsync_obj_drop(mutex, "create_mutex:post_get_fd");
    if (error != 0)
        return (error);
    *fdp = fd;
    return (0);
}

static int
ntsync_create_event(struct ntsync_device *dev, void *uarg, int *fdp)
{
    struct ntsync_event_args args;
    struct ntsync_obj *event;
    int fd, error;

    if (ntsync_kern_copyin(uarg, &args, sizeof(args)) != 0)
        return (EFAULT);

    event = ntsync_alloc_obj(dev, NTSYNC_TYPE_EVENT);
    /*
     * Normalize args.manual and args.signaled to 0/1.  The wire ABI uses
     * __u32 for both fields but internally they are stored as bool.  A
     * caller passing manual=2 would read back manual=1 after the bool
     * truncation, creating an ABI inconsistency.  Normalizing on creation
     * ensures read-back is always identical to the written value.
     */
    event->u.event.manual  = (args.manual  != 0);
    event->u.event.signaled = (args.signaled != 0);
    error = ntsync_obj_get_fd(curthread, event, &fd);
    ntsync_obj_drop(event, "create_event:post_get_fd");
    if (error != 0)
        return (error);
    *fdp = fd;
    return (0);
}

/* ---------------------- Object fileops ------------------------------- */

static int
ntsync_obj_close(struct file *fp, struct thread *td)
{
    struct ntsync_obj *obj = fp->f_data;

    if (obj != NULL) {
        struct ntsync_device *dev = obj->dev;
        struct ntsync_q_entry *entry, *tmp;

        /*
         * Acquire device-level wait_all lock first (if present), then
         * object lock.  Lock ordering: wait_all_mtx -> obj->mtx.
         */
        if (dev != NULL)
            mtx_lock(&dev->wait_all_mtx);
        mtx_lock(&obj->mtx);

        /* Mark deleted so wait helpers abort rather than consume state. */
        obj->deleted = 1;
        NTSYNC_DPRINTF("ntsync: obj_close %p id=%ju deleted=1, waking waiters\n",
            obj, (uintmax_t)obj->id);

#ifdef NTSYNC_DEBUG
        {
            struct ntsync_q_entry *e;
            int cnt = 0;
            TAILQ_FOREACH(e, &obj->any_waiters, link) {
                NTSYNC_DPRINTF("ntsync: obj_close any_waiter[%d] obj=%p id=%ju -> q=%p index=%u\n",
                       cnt, obj, (uintmax_t)obj->id, e->q, e->index);
                cnt++;
            }
            if (cnt == 0)
                NTSYNC_DPRINTF("ntsync: obj_close any_waiters empty for obj=%p id=%ju\n",
                       obj, (uintmax_t)obj->id);
            cnt = 0;
            TAILQ_FOREACH(e, &obj->all_waiters, link) {
                NTSYNC_DPRINTF("ntsync: obj_close all_waiter[%d] obj=%p id=%ju -> q=%p index=%u\n",
                       cnt, obj, (uintmax_t)obj->id, e->q, e->index);
                cnt++;
            }
            if (cnt == 0)
                NTSYNC_DPRINTF("ntsync: obj_close all_waiters empty for obj=%p id=%ju\n",
                       obj, (uintmax_t)obj->id);
        }
#endif

        /*
         * Wake all registered waiters with a deletion/abort result.
         * ntsync_q_signal_deleted sets q->aborted=true, q->signaled=index
         * and calls cv_signal.  A single pass over both waiter lists
         * is sufficient.  ntsync_obj_notify fires poll/kqueue subscribers.
         */
        TAILQ_FOREACH_SAFE(entry, &obj->any_waiters, link, tmp)
            if (entry->q != NULL)
                ntsync_q_signal_deleted(entry->q, entry->index);
        TAILQ_FOREACH_SAFE(entry, &obj->all_waiters, link, tmp)
            if (entry->q != NULL)
                ntsync_q_signal_deleted(entry->q, entry->index);
        ntsync_obj_notify(obj);

        if (dev != NULL)
            mtx_unlock(&dev->wait_all_mtx);

        /*
         * Wait for active waiters to drain.  Waiters increment
         * obj->active_waiters when they register and decrement it when
         * they unregister.  We hold obj->mtx throughout.
         *
         * Use cv_timedwait with a 5-second deadline instead of the
         * original unbounded cv_wait.  In correct operation the drain
         * completes in microseconds (the abort/wakeup path guarantees
         * every sleeper calls cv_signal on their q->cv, wakes, removes
         * itself from the waiter list, and decrements active_waiters
         * before touching anything else).  The timeout guards against a
         * future bug where a waiter fails to decrement active_waiters,
         * which would otherwise hang the closing thread permanently with
         * no way to interrupt it — even SIGKILL cannot deliver while
         * sleeping in cv_wait.  If the drain stalls we emit a warning and
         * a KASSERT so INVARIANTS kernels catch the regression; in
         * production builds we break out rather than hang forever.
         */
        {
            int _drain_iter = 0;
            while (obj->active_waiters > 0) {
                int _r = cv_timedwait(&obj->waiters_cv, &obj->mtx, hz * 5);
                if (_r == EWOULDBLOCK) {
                    printf("ntsync: obj_close drain timeout "
                        "obj=%p active_waiters=%d (iter=%d) -- "
                        "waiter list invariant broken\n",
                        obj, obj->active_waiters, ++_drain_iter);
                    KASSERT(0, ("ntsync: active_waiters drain stalled"));
                    break;
                }
            }
        }

        mtx_unlock(&obj->mtx);
    }

    /*
     * Drop our reference (may free the object if this was the last ref).
     * Guard against NULL: in production builds ntsync_obj_drop expands to
     * ntsync_obj_unref, which has no NULL guard and would dereference obj
     * immediately.  The if (obj != NULL) block above implies NULL is at
     * least theoretically possible, so protect the drop to match.
     */
    if (obj != NULL)
        ntsync_obj_drop(obj, "obj_close:final_unref");
    fp->f_data = NULL;
    return (0);
}

static int
ntsync_obj_ioctl(struct file *fp, u_long cmd, void *data, struct ucred *cred, struct thread *td)
{
    struct ntsync_obj *obj = fp->f_data;
    /*
     * Recover the original user-space pointer from the raw syscall args.
     * kern_ioctl only copies the kernel buffer (data) back to user space
     * when the ioctl returns 0.  Functions that must deliver data on
     * non-zero returns (EOWNERDEAD) copy directly to this pointer instead.
     */
    void *udata = ntsync_udata_ptr(td);

    switch (cmd) {
    case NTSYNC_IOC_SEM_RELEASE:
        return (ntsync_sem_release(obj, data));
    case NTSYNC_IOC_SEM_READ:
        return (ntsync_sem_read(obj, data));
    case NTSYNC_IOC_MUTEX_UNLOCK:
        return (ntsync_mutex_unlock(obj, data));
    case NTSYNC_IOC_MUTEX_KILL:
        return (ntsync_mutex_kill(obj, data));
    case NTSYNC_IOC_MUTEX_READ:
        return (ntsync_mutex_read(obj, data, udata));
    case NTSYNC_IOC_EVENT_SET:
        return (ntsync_event_set_reset(obj, data, true, false));
    case NTSYNC_IOC_EVENT_RESET:
        return (ntsync_event_set_reset(obj, data, false, false));
    case NTSYNC_IOC_EVENT_PULSE:
        return (ntsync_event_set_reset(obj, data, true, true));
    case NTSYNC_IOC_EVENT_READ:
        return (ntsync_event_read(obj, data));
    default:
        return (ENOTTY);
    }
}

static int
ntsync_obj_poll(struct file *fp, int events, struct ucred *cred, struct thread *td)
{
    struct ntsync_obj *obj = fp->f_data;
    int revents = 0;

    if (events & (POLLIN | POLLRDNORM)) {
        bool signaled = false;
        mtx_lock(&obj->mtx);
        switch (obj->type) {
        case NTSYNC_TYPE_SEM:   signaled = (obj->u.sem.count > 0); break;
        /*
         * For a mutex we report POLLIN only when the mutex is completely
         * unowned (owner == 0).  The fully correct test would also accept
         * an already-owned mutex whose owner matches the calling process
         * (recursive acquisition), but poll(2)/kevent(2) carry no owner
         * hint — there is no way to pass one through the VFS poll ABI.
         * This is the same deliberate simplification used by the Linux
         * ntsync driver.  Wine does not rely on poll/kqueue for mutex fds;
         * it always uses WAIT_ANY/WAIT_ALL which do carry an owner field.
         */
        case NTSYNC_TYPE_MUTEX: signaled = (obj->u.mutex.owner == 0); break;
        case NTSYNC_TYPE_EVENT: signaled = obj->u.event.signaled; break;
        default: break;
        }
        if (signaled)
            revents |= (POLLIN | POLLRDNORM);
        else {
            /*
             * Mark that a poll watcher has registered so ntsync_obj_notify
             * knows to call selwakeup.  Set before selrecord to avoid a
             * race where the object becomes signaled between the two writes.
             */
            obj->notify_hint = 1;
            selrecord(td, &obj->selinfo);
        }
        mtx_unlock(&obj->mtx);
    }
    return (revents);
}

static int
ntsync_kqfilter_read(struct knote *kn, long hint)
{
    struct ntsync_obj *obj = kn->kn_hook;
    bool signaled = false;

    mtx_assert(&obj->mtx, MA_OWNED);  /* held by KNOTE_LOCKED */
    switch (obj->type) {
    case NTSYNC_TYPE_SEM:   signaled = (obj->u.sem.count > 0); break;
    /*
     * See the comment in ntsync_obj_poll: owner == 0 is an intentional
     * simplification.  The kqueue ABI provides no owner hint, so recursive
     * mutex ownership cannot be detected here.  Wine does not use kqueue
     * on mutex fds, so this has no practical impact.
     */
    case NTSYNC_TYPE_MUTEX: signaled = (obj->u.mutex.owner == 0); break;
    case NTSYNC_TYPE_EVENT: signaled = obj->u.event.signaled; break;
    default: break;
    }
    kn->kn_data = signaled ? 1 : 0;
    return (signaled ? 1 : 0);
}

static void
ntsync_kqfilter_detach(struct knote *kn)
{
    struct ntsync_obj *obj = kn->kn_hook;
    knlist_remove(&obj->knlist, kn, 0);
    ntsync_obj_drop(obj, "kqfilter:detach");
}

static int
ntsync_obj_kqfilter(struct file *fp, struct knote *kn)
{
    struct ntsync_obj *obj = fp->f_data;

    switch (kn->kn_filter) {
    case EVFILT_READ:
        kn->kn_fop = &ntsync_kqread_filtops;
        kn->kn_hook = obj;
        ntsync_obj_ref(obj);
        /*
         * Mark that a kqueue watcher is registering before inserting into
         * the knlist, so ntsync_obj_notify cannot observe hint=0 while a
         * live knote exists in obj->knlist.
         */
        obj->notify_hint = 1;
        knlist_add(&obj->knlist, kn, 0);
        return (0);
    default:
        return (EINVAL);
    }
}

/* ---------------------- Control device ops -------------------------- */

static int
ntsync_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
    return (0);
}

static int
ntsync_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
    return (0);
}

static int
ntsync_ioctl(struct cdev *cdev, u_long cmd, caddr_t data, int fflag, struct thread *td)
{
    struct ntsync_device *dev = cdev->si_drv1;
    /*
     * Save the original user-space pointer before kern_ioctl replaces it
     * with a kernel buffer.  WAIT_ANY and WAIT_ALL must write the output
     * index directly to this address when returning non-zero (EOWNERDEAD),
     * because kern_ioctl discards the kernel buffer on error returns.
     */
    void *udata = ntsync_udata_ptr(td);
    int error, fd;

    switch (cmd) {
    case NTSYNC_IOC_CREATE_SEM:
        error = ntsync_create_sem(dev, data, &fd);
        if (error != 0)
            return (error);
        td->td_retval[0] = fd;
        return (0);
    case NTSYNC_IOC_CREATE_MUTEX:
        error = ntsync_create_mutex(dev, data, &fd);
        if (error != 0)
            return (error);
        td->td_retval[0] = fd;
        return (0);
    case NTSYNC_IOC_CREATE_EVENT:
        error = ntsync_create_event(dev, data, &fd);
        if (error != 0)
            return (error);
        td->td_retval[0] = fd;
        return (0);
    case NTSYNC_IOC_WAIT_ANY:
        return (ntsync_wait_any(dev, data, udata));
    case NTSYNC_IOC_WAIT_ALL:
        return (ntsync_wait_all(dev, data, udata));
    default:
        return (ENOTTY);
    }
}

/* ---------------------- Module glue --------------------------------- */

/*
 * UMA init/fini callbacks for ntsync_q.
 *
 * FreeBSD UMA distinguishes two levels of callback:
 *
 *   init / fini  — called ONCE per backing-memory slot when the slab is
 *                  carved out of (or returned to) the system.  The mutex
 *                  and condvar inside each ntsync_q are initialised here,
 *                  so they persist across many alloc/free cycles without
 *                  ever being destroyed and re-created.
 *
 *   ctor / dtor  — called on every uma_zalloc / uma_zfree.  We leave
 *                  these NULL; per-call field reset is done in
 *                  ntsync_q_init() instead.
 *
 * Consequence: mtx_init, cv_init, cv_destroy, mtx_destroy are no longer
 * called on the hot wait/wake path.  Under a running game the driver
 * issues thousands of WAIT_ANY/WAIT_ALL calls per second; removing four
 * lock-lifecycle operations per call meaningfully reduces both average
 * latency and variance (1% lows).
 *
 * Safety invariant: every code path that holds q->lock releases it
 * before uma_zfree, and every thread sleeping on q->cv has woken and
 * left the sleep queue before uma_zfree.  The recycled object therefore
 * presents an unlocked mutex and a quiescent condvar to the next caller,
 * exactly as if mtx_init / cv_init had just been called.
 */
static int
ntsync_q_uma_init(void *mem, int size, int flags)
{
    struct ntsync_q *q = mem;
    mtx_init(&q->lock, "ntsync_q", NULL, MTX_DEF);
    cv_init(&q->cv, "ntsync_q_cv");
    return (0);
}

static void
ntsync_q_uma_fini(void *mem, int size)
{
    struct ntsync_q *q = mem;
    cv_destroy(&q->cv);
    mtx_destroy(&q->lock);
}

static int
ntsync_modevent(module_t mod, int type, void *data)
{
    int error = 0;

    switch (type) {
    case MOD_LOAD: {
        struct cdev *cdev;

#ifdef NTSYNC_DEBUG
        ntsync_obj_count = counter_u64_alloc(M_WAITOK);
#endif
        ntsync_active_q_count = counter_u64_alloc(M_WAITOK);
        /* M_WAITOK: counter_u64_alloc sleeps until memory is available;
         * it will not return NULL. */
        ntsync_q_zone = uma_zcreate("ntsync_q", NTSYNC_Q_MAX_SIZE,
            NULL, NULL, ntsync_q_uma_init, ntsync_q_uma_fini,
            UMA_ALIGN_PTR, 0);
        if (ntsync_q_zone == NULL) {
            counter_u64_free(ntsync_active_q_count);
            ntsync_active_q_count = NULL;
#ifdef NTSYNC_DEBUG
            counter_u64_free(ntsync_obj_count);
            ntsync_obj_count = NULL;
#endif
            error = ENOMEM;
            break;
        }
        ntsync_dev = malloc(sizeof(*ntsync_dev), M_NTSYNC, M_WAITOK | M_ZERO);
        mtx_init(&ntsync_dev->wait_all_mtx, "ntsync_wait_all", NULL, MTX_DEF);
        ntsync_dev->refcnt = 1;  /* module holds one reference */
        cdev = make_dev(&ntsync_cdevsw, 0, UID_ROOT, GID_WHEEL, 0666, NTSYNC_NAME);
        if (cdev == NULL) {
            mtx_destroy(&ntsync_dev->wait_all_mtx);
            free(ntsync_dev, M_NTSYNC);
            ntsync_dev = NULL;
            uma_zdestroy(ntsync_q_zone);
            ntsync_q_zone = NULL;
            counter_u64_free(ntsync_active_q_count);
            ntsync_active_q_count = NULL;
#ifdef NTSYNC_DEBUG
            counter_u64_free(ntsync_obj_count);
            ntsync_obj_count = NULL;
#endif
            error = ENXIO;
            break;
        }

        cdev->si_drv1 = ntsync_dev;
        ntsync_dev->cdev = cdev;
        printf("ntsync: loaded\n");
        break;
    }
    case MOD_UNLOAD:
        if (ntsync_dev != NULL) {
             /*
              * SECURITY FIX: Reject unload if object file descriptors are still alive.
              * Anonymous descriptors created via falloc() do not hold a module
              * reference. We must manually check the device reference count to
              * prevent a Use-After-Free panic when the user eventually closes
              * remaining object FDs after the module code has been unmapped.
              */
             if (atomic_load_int(&ntsync_dev->refcnt) > 1) {
                 return (EBUSY);
            }
            /*
             * Prevent any new opens/ioctls on the control device.
             * Object fds that are already open remain valid; their ioctls
             * access obj->dev (the ntsync_device pointer) which is kept
             * alive by the per-object device reference until the last fd
             * is closed.
             */
            destroy_dev(ntsync_dev->cdev);
            ntsync_dev->cdev = NULL;
            /*
             * Drop the module's reference.  If no object fds are open this
             * will be the final drop and will free the device (including
             * destroying wait_all_mtx).  If object fds remain open the
             * device lives until the last ntsync_obj_unref drops its ref.
             */
            ntsync_dev_drop(ntsync_dev);
            ntsync_dev = NULL;
        }
        if (ntsync_q_zone != NULL) {
            /*
             * Wait for all threads currently sleeping inside
             * ntsync_schedule (which hold a live ntsync_q pointer from
             * ntsync_q_zone) to wake up and call counter_u64_add(-1)
             * and uma_zfree before we destroy the zone.  Threads are
             * woken by the object-close / abort path; once ntsync_dev_drop
             * frees the device all such threads should drain quickly.
             *
             * Poll at 10 ms intervals for up to 5 seconds.  Use
             * counter_u64_fetch() for an accurate aggregate across all
             * CPU-local slots; a plain volatile read would lack the
             * acquire barrier needed for correctness on ARM64.
             */
            for (int _i = 0; _i < 500 &&
                counter_u64_fetch(ntsync_active_q_count) != 0; _i++)
                pause("ntsyncunld", hz / 100);
            if (counter_u64_fetch(ntsync_active_q_count) != 0)
                printf("ntsync: WARNING: active waiter(s) at unload -- "
                    "zone destroy may panic\n");
            uma_zdestroy(ntsync_q_zone);
            ntsync_q_zone = NULL;
        }
        if (ntsync_obj_count != NULL) {
#ifdef NTSYNC_DEBUG
            counter_u64_free(ntsync_obj_count);
            ntsync_obj_count = NULL;
#endif
        }
        if (ntsync_active_q_count != NULL) {
            counter_u64_free(ntsync_active_q_count);
            ntsync_active_q_count = NULL;
        }
        printf("ntsync: unloaded\n");
        break;
    default:
        error = EOPNOTSUPP;
        break;
    }
    return (error);
}

DEV_MODULE(ntsync, ntsync_modevent, NULL);
MODULE_VERSION(ntsync, 1);
