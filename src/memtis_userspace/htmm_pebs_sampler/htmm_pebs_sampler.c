#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/perf_event.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/kfifo.h>
#include <linux/workqueue.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/highmem.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/llist.h>
#include <linux/irq_work.h>
#include <linux/slab.h>

/* x86 raw event codes (match memtis htmm.h) */
#define DRAM_LLC_LOAD_MISS  0x1d3
#define REMOTE_DRAM_LLC_LOAD_MISS   0x2d3
#define NVM_LLC_LOAD_MISS   0x80d1
#define ALL_STORES          0x82d0

enum samp_event_kind {
    S_EV_DRAMREAD = 0,
    S_EV_NVMREAD  = 1,
    S_EV_MEMWRITE = 2,
    S_EV_CXLREAD  = 3,
    S_EV_COUNT
};

struct samp_rec {
    u32 pid;
    u32 tid;
    u64 addr;
    u32 ev_kind;
    u64 data_src; /* PERF_SAMPLE_DATA_SRC */
};

struct ev_ctx {
    int ev_kind;
};

struct cpu_ctx {
    struct perf_event *events[S_EV_COUNT];
    struct kfifo fifo;
    spinlock_t fifo_lock; /* protects fifo in hardirq */
    struct work_struct work;
    atomic64_t dropped;
    struct llist_head nmi_queue; /* lockless queue for NMI -> work */
    struct irq_work iw;          /* NMI-safe trigger to schedule work */
};

static DEFINE_PER_CPU(struct cpu_ctx, cpu_ctxs);

struct samp_rec_node {
    struct llist_node node;
    struct samp_rec rec;
};

static struct kmem_cache *samp_node_cachep;

static void iw_callback(struct irq_work *iw);

static void iw_callback(struct irq_work *iw)
{
    struct cpu_ctx *ctx = container_of(iw, struct cpu_ctx, iw);
    schedule_work(&ctx->work);
}

/* module params */
static int sample_pid = -1; /* -1: per-cpu sampling; >=0: per-task sampling */
module_param(sample_pid, int, 0644);
MODULE_PARM_DESC(sample_pid, "Target pid (-1 for per-cpu events)");

static unsigned int llc_sample_period = 199;
module_param(llc_sample_period, uint, 0644);
MODULE_PARM_DESC(llc_sample_period, "LLC miss event sample period");

static unsigned int store_sample_period = 100003;
module_param(store_sample_period, uint, 0644);
MODULE_PARM_DESC(store_sample_period, "Store instruction sample period");

static bool enable_dram = true;
module_param(enable_dram, bool, 0644);
MODULE_PARM_DESC(enable_dram, "Enable DRAM LLC load miss sampling");

static bool enable_nvm = false;
module_param(enable_nvm, bool, 0644);
MODULE_PARM_DESC(enable_nvm, "Enable NVM LLC load miss sampling");

static bool enable_cxl = false;
module_param(enable_cxl, bool, 0644);
MODULE_PARM_DESC(enable_cxl, "Enable CXL(remote DRAM) LLC load miss sampling");

static bool enable_store = true;
module_param(enable_store, bool, 0644);
MODULE_PARM_DESC(enable_store, "Enable ALL_STORES sampling");

static int precise_ip = 1; /* 1 or 2 */
module_param(precise_ip, int, 0644);
MODULE_PARM_DESC(precise_ip, "PEBS precise_ip level (1 or 2)");

static unsigned int fifo_order = 12; /* 2^12 entries (~4096) */
module_param(fifo_order, uint, 0644);
MODULE_PARM_DESC(fifo_order, "Per-CPU kfifo order (power-of-two entries)");

static inline bool valid_user_va(unsigned long addr)
{
#ifdef CONFIG_X86_64
    return addr && addr < TASK_SIZE_MAX;
#else
    return addr && addr < TASK_SIZE;
#endif
}

static const char *ev_kind_to_str(int kind)
{
    switch (kind) {
    case S_EV_DRAMREAD:
        return "DRAMREAD";
    case S_EV_NVMREAD:
        return "NVMREAD";
    case S_EV_MEMWRITE:
        return "MEMWRITE";
    case S_EV_CXLREAD:
        return "CXLREAD";
    default:
        return "UNKNOWN";
    }
}

static int get_nid_for_address(pid_t pid, unsigned long address)
{
    struct pid *pid_struct = NULL;
    struct task_struct *task = NULL;
    struct mm_struct *mm = NULL;
    struct page *page = NULL;
    int locked = 1;
    int nid = -EINVAL;
    long got;

    if (!valid_user_va(address))
        return -EINVAL;

    if (pid >= 0) {
        pid_struct = find_get_pid(pid);
        task = pid_struct ? get_pid_task(pid_struct, PIDTYPE_PID) : NULL;
    } else {
        task = current;
        get_task_struct(task);
    }

    if (!task)
        goto out_put;

    mm = get_task_mm(task);
    if (!mm)
        goto out_put;

    /*
     * 传入 locked 指针时，调用方必须持有 mmap_read_lock(mm) 且将 *locked 设为 1，
     * 否则 gup 会触发 BUG_ON(*locked != 1)。
     */
    mmap_read_lock(mm);
    got = get_user_pages_remote(mm, address, 1,
                                FOLL_GET, &page, NULL, &locked);
    if (locked)
        mmap_read_unlock(mm);
    mmput(mm);

    if (got == 1 && page) {
        page = compound_head(page);
        nid = page_to_nid(page);
        put_page(page);
    }

out_put:
    if (task)
        put_task_struct(task);
    if (pid_struct)
        put_pid(pid_struct);
    return nid;
}

static void process_fifo_work(struct work_struct *work)
{
    struct cpu_ctx *ctx = container_of(work, struct cpu_ctx, work);
    struct samp_rec rec;
    unsigned long flags;
    struct llist_node *head;
    /* 先处理 NMI 无锁队列中的样本 */
    head = llist_del_all(&ctx->nmi_queue);
    head = llist_reverse_order(head);
    while (head) {
        struct samp_rec_node *n = llist_entry(head, struct samp_rec_node, node);
        head = head->next;
		if (valid_user_va(n->rec.addr)) {
			int nid = get_nid_for_address(n->rec.pid, (unsigned long)n->rec.addr);
			trace_printk("htmm_pebs_sampler: pid=%u vaddr=%#llx ev=%s data_src=%#llx nid=%d\n",
			     n->rec.pid, n->rec.addr, ev_kind_to_str(n->rec.ev_kind), n->rec.data_src, nid);
		}
        kmem_cache_free(samp_node_cachep, n);
    }

    for (;;) {
        spin_lock_irqsave(&ctx->fifo_lock, flags);
        if (!kfifo_len(&ctx->fifo)) {
            spin_unlock_irqrestore(&ctx->fifo_lock, flags);
            break;
        }
        if (!kfifo_out(&ctx->fifo, &rec, sizeof(rec))) {
            spin_unlock_irqrestore(&ctx->fifo_lock, flags);
            break;
        }
        spin_unlock_irqrestore(&ctx->fifo_lock, flags);

		if (valid_user_va(rec.addr)) {
			int nid = get_nid_for_address(rec.pid, (unsigned long)rec.addr);
			trace_printk("htmm_pebs_sampler: pid=%u vaddr=%#llx ev=%s data_src=%#llx nid=%d\n",
			     rec.pid, rec.addr, ev_kind_to_str(rec.ev_kind), rec.data_src, nid);
		}
    }
}

static void pebs_overflow(struct perf_event *event,
                          struct perf_sample_data *data,
                          struct pt_regs *regs)
{
    struct ev_ctx *ectx = event->overflow_handler_context;
    struct cpu_ctx *ctx = this_cpu_ptr(&cpu_ctxs);
    struct samp_rec_node *n;

    if (!data)
        return;

    /* NMI 上下文：仅做无锁入队 + irq_work 触发 */
    if (!data->addr)
        return; /* 没有有效地址的样本直接丢弃，减轻热路径负担 */

    n = kmem_cache_alloc(samp_node_cachep, GFP_ATOMIC);
    if (!n) {
        atomic64_inc(&ctx->dropped);
        return;
    }

    n->rec.pid = (sample_pid >= 0) ? (u32)sample_pid : (u32)current->pid;
    n->rec.tid = n->rec.pid; /* 内核 overflow 回调的 perf_sample_data 无 tid 字段，退回当前 pid */
    n->rec.addr = data->addr;
    n->rec.ev_kind = ectx ? ectx->ev_kind : 0;
    n->rec.data_src = data->data_src.val;

    llist_add(&n->node, &ctx->nmi_queue);
    irq_work_queue(&ctx->iw);
}

static __u64 raw_config_for_kind(int kind)
{
    switch (kind) {
    case S_EV_DRAMREAD:
        return DRAM_LLC_LOAD_MISS;
    case S_EV_NVMREAD:
        return NVM_LLC_LOAD_MISS;
    case S_EV_MEMWRITE:
        return ALL_STORES;
    case S_EV_CXLREAD:
        return REMOTE_DRAM_LLC_LOAD_MISS;
    default:
        return 0;
    }
}

static int setup_event_on_cpu(int cpu, int kind)
{
    struct perf_event_attr attr;
    struct perf_event *evt;
    struct task_struct *task = NULL;
    struct pid *pid_ref = NULL;
    struct cpu_ctx *cctx = per_cpu_ptr(&cpu_ctxs, cpu);
    static struct ev_ctx contexts[S_EV_COUNT];

    memset(&attr, 0, sizeof(attr));
    attr.type = PERF_TYPE_RAW;
    attr.size = sizeof(attr);
    attr.config = raw_config_for_kind(kind);
    attr.sample_type = PERF_SAMPLE_IP | PERF_SAMPLE_ADDR | PERF_SAMPLE_TID | PERF_SAMPLE_DATA_SRC;
    attr.precise_ip = precise_ip;
    attr.exclude_kernel = 1;
    attr.exclude_hv = 1;
    attr.exclude_callchain_kernel = 1;
    attr.exclude_callchain_user = 1;
    if (kind == S_EV_MEMWRITE)
        attr.sample_period = store_sample_period;
    else
        attr.sample_period = llc_sample_period;

    contexts[kind].ev_kind = kind;

    if (sample_pid >= 0) {
        pid_ref = find_get_pid(sample_pid);
        task = get_pid_task(pid_ref, PIDTYPE_PID);
        if (!task) {
            if (pid_ref)
                put_pid(pid_ref);
            return -ESRCH;
        }
    }

    /*
     * 当指定了 task 时，perf_event 会监控该 task 在所有 CPU 上的执行。
     * cpu 参数用于指定硬件 PMU 计数器所在的 CPU（对于 CPU-local 计数器）。
     * 因此，即使指定了 sample_pid，我们仍然在每个 CPU 上创建事件，
     * 这样可以在每个 CPU 上都有独立的硬件计数器来采样该进程。
     */
    evt = perf_event_create_kernel_counter(&attr, cpu, task,
                                           pebs_overflow, &contexts[kind]);
    if (IS_ERR(evt)) {
        if (task)
            put_task_struct(task);
        if (pid_ref)
            put_pid(pid_ref);
        return PTR_ERR(evt);
    }
    cctx->events[kind] = evt;
    perf_event_enable(evt);
    if (task)
        put_task_struct(task);
    if (pid_ref)
        put_pid(pid_ref);
    return 0;
}

static void teardown_event_on_cpu(int cpu, int kind)
{
    struct cpu_ctx *cctx = per_cpu_ptr(&cpu_ctxs, cpu);
    struct perf_event *evt = cctx->events[kind];
    if (!evt)
        return;
    perf_event_disable(evt);
    perf_event_release_kernel(evt);
    cctx->events[kind] = NULL;
}

static int setup_cpu(int cpu)
{
    int ret = 0;
    struct cpu_ctx *cctx = per_cpu_ptr(&cpu_ctxs, cpu);
    int fifo_elems = 1 << fifo_order;

    INIT_WORK(&cctx->work, process_fifo_work);
    init_llist_head(&cctx->nmi_queue);
    init_irq_work(&cctx->iw, iw_callback);
    spin_lock_init(&cctx->fifo_lock);
    atomic64_set(&cctx->dropped, 0);

    if (kfifo_alloc(&cctx->fifo, fifo_elems * sizeof(struct samp_rec), GFP_KERNEL))
        return -ENOMEM;

    if (enable_dram) {
        ret = setup_event_on_cpu(cpu, S_EV_DRAMREAD);
        if (ret) goto err;
    }
    if (enable_nvm) {
        ret = setup_event_on_cpu(cpu, S_EV_NVMREAD);
        if (ret) goto err;
    }
    if (enable_cxl) {
        ret = setup_event_on_cpu(cpu, S_EV_CXLREAD);
        if (ret) goto err;
    }
    if (enable_store) {
        ret = setup_event_on_cpu(cpu, S_EV_MEMWRITE);
        if (ret) goto err;
    }
    return 0;
err:
    teardown_event_on_cpu(cpu, S_EV_DRAMREAD);
    teardown_event_on_cpu(cpu, S_EV_NVMREAD);
    teardown_event_on_cpu(cpu, S_EV_CXLREAD);
    teardown_event_on_cpu(cpu, S_EV_MEMWRITE);
    kfifo_free(&cctx->fifo);
    return ret;
}

static void teardown_cpu(int cpu)
{
    struct cpu_ctx *cctx = per_cpu_ptr(&cpu_ctxs, cpu);
    struct llist_node *head;
    struct samp_rec_node *n;
    
    /* 先停止事件，避免新的 NMI 回调 */
    teardown_event_on_cpu(cpu, S_EV_DRAMREAD);
    teardown_event_on_cpu(cpu, S_EV_NVMREAD);
    teardown_event_on_cpu(cpu, S_EV_CXLREAD);
    teardown_event_on_cpu(cpu, S_EV_MEMWRITE);
    
    /* 先同步 irq_work，避免其在 cancel 之后再次调度 work */
    irq_work_sync(&cctx->iw);

    /* 再取消工作队列，确保所有已调度的 work 运行完毕 */
    cancel_work_sync(&cctx->work);
    
    /* 清理 NMI 队列中剩余的节点 */
    head = llist_del_all(&cctx->nmi_queue);
    while (head) {
        n = llist_entry(head, struct samp_rec_node, node);
        head = head->next;
        if (samp_node_cachep)
            kmem_cache_free(samp_node_cachep, n);
    }
    
    kfifo_free(&cctx->fifo);
}

static int __init htmm_pebs_sampler_init(void)
{
    int cpu, ret;

#ifndef CONFIG_X86
    pr_err("htmm_pebs_sampler: only supported on x86 for now\n");
    return -ENOTSUPP;
#endif

    samp_node_cachep = kmem_cache_create("htmm_pebs_samp_node",
                                         sizeof(struct samp_rec_node), 0,
                                         SLAB_HWCACHE_ALIGN, NULL);
    if (!samp_node_cachep)
        return -ENOMEM;

    cpus_read_lock();
    for_each_online_cpu(cpu) {
        ret = setup_cpu(cpu);
        if (ret) {
            pr_err("htmm_pebs_sampler: setup cpu %d failed: %d\n", cpu, ret);
            /* rollback */
            for_each_online_cpu(cpu) teardown_cpu(cpu);
            cpus_read_unlock();
            kmem_cache_destroy(samp_node_cachep);
            samp_node_cachep = NULL;
            return ret;
        }
    }
    cpus_read_unlock();

    pr_info("htmm_pebs_sampler: loaded (pid=%d, dram=%d nvm=%d cxl=%d store=%d)\n",
            sample_pid, enable_dram, enable_nvm, enable_cxl, enable_store);
    return 0;
}

static void __exit htmm_pebs_sampler_exit(void)
{
    int cpu;
    cpus_read_lock();
    for_each_online_cpu(cpu)
        teardown_cpu(cpu);
    cpus_read_unlock();
    if (samp_node_cachep) {
        kmem_cache_destroy(samp_node_cachep);
        samp_node_cachep = NULL;
    }
    pr_info("htmm_pebs_sampler: unloaded\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("memtis");
MODULE_DESCRIPTION("PEBS-based memory access sampler (out-of-tree)");

module_init(htmm_pebs_sampler_init);
module_exit(htmm_pebs_sampler_exit);


