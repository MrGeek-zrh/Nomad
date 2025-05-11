/*
 * memory access sampling for hugepage-aware tiered memory management.
 */
#include "asm/pgtable.h"
#include "linux/compiler.h"
#include <linux/memcontrol.h>
#include <linux/mempolicy.h>
#include <linux/sched.h>
#include <linux/perf_event.h>
#include <linux/sched/cputime.h>

#include "../kernel/events/internal.h"

#include <linux/htmm.h>
#include <linux/types.h>

#include "linux/gfp.h"
#include "linux/jiffies.h"
#include <linux/sort.h>
#include <linux/vmalloc.h>

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/mmu_notifier.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/numa.h>
#include <linux/pagewalk.h>
#include <linux/hugetlb.h>
#include <linux/page_idle.h>
#include <linux/migrate.h>
#include <linux/kallsyms.h>
#include <linux/mmzone.h>
#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/sched/signal.h>
#include <linux/sched/smt.h>
#include <linux/sched/isolation.h>

#define pr_fmt(fmt) "phmm: " fmt

bool numa_has_cpu(int nid)
{
    return node_state(nid, N_CPU);
}

struct task_struct *access_sampling = NULL;
struct perf_event ***mem_event;

static bool valid_va(unsigned long addr)
{
    if (!(addr >> (PGDIR_SHIFT + 9)) && addr != 0)
	return true;
    else
	return false;
}

static __u64 get_pebs_event(enum events e)
{
    switch (e) {
	case DRAMREAD:
	    return DRAM_LLC_LOAD_MISS;
	case NVMREAD:
	    if (htmm_cxl_mode)
		return NVM_LLC_LOAD_MISS;
	    else
		return N_HTMMEVENTS;
	case MEMWRITE:
	    return ALL_STORES;
	case CXLREAD:
	    if (htmm_cxl_mode)
		return REMOTE_DRAM_LLC_LOAD_MISS;
	    else
		return N_HTMMEVENTS;
	default:
	    return N_HTMMEVENTS;
    }
}

static int __perf_event_open(__u64 config, __u64 config1, __u64 cpu,
	__u64 type, __u32 pid)
{
    struct perf_event_attr attr;
    struct file *file;
    int event_fd, __pid;

    memset(&attr, 0, sizeof(struct perf_event_attr));

    attr.type = PERF_TYPE_RAW;
    attr.size = sizeof(struct perf_event_attr);
    attr.config = config;
    attr.config1 = config1;
    if (config == ALL_STORES)
	attr.sample_period = htmm_inst_sample_period;
    else
	attr.sample_period = get_sample_period(0);
    attr.sample_type = PERF_SAMPLE_IP | PERF_SAMPLE_TID | PERF_SAMPLE_ADDR;
    attr.disabled = 0;
    attr.exclude_kernel = 1;
    attr.exclude_hv = 1;
    attr.exclude_callchain_kernel = 1;
    attr.exclude_callchain_user = 1;
    attr.precise_ip = 0;
    attr.enable_on_exec = 1;

    if (pid == 0)
	__pid = -1;
    else
	__pid = pid;
	
    event_fd = htmm__perf_event_open(&attr, __pid, cpu, -1, 0);
    //event_fd = htmm__perf_event_open(&attr, -1, cpu, -1, 0);
    if (event_fd <= 0) {
	    printk("[error htmm__perf_event_open failure] event_fd: %d, config %llx, config1 %llx\n",
		   event_fd, config, config1);
	    return -1;
    }

    file = fget(event_fd);
    if (!file) {
	printk("invalid file\n");
	return -1;
    }
    mem_event[cpu][type] = fget(event_fd)->private_data;
    return 0;
}

static int pebs_init(pid_t pid, int node)
{
    int cpu, event;

    mem_event = kzalloc(sizeof(struct perf_event **) * num_online_cpus(), GFP_KERNEL);
    for_each_online_cpu (cpu) {
	mem_event[cpu] = kzalloc(sizeof(struct perf_event *) * N_HTMMEVENTS, GFP_KERNEL);
    }
    
    printk("pebs_init\n");

    for_each_online_cpu (cpu) {
	    for (event = 0; event < N_HTMMEVENTS; event++) {
		    if (get_pebs_event(event) == N_HTMMEVENTS) {
			    mem_event[cpu][event] = NULL;
			    continue;
		    }

		    if (__perf_event_open(get_pebs_event(event), 0, cpu, event,
					  pid))
			    return -1;
		    if (htmm__perf_event_init(mem_event[cpu][event],
					      BUFFER_SIZE))
			    return -1;
	    }
    }

    return 0;
}

static void pebs_disable(void)
{
	int cpu, event;
	printk("pebs disable\n");
	for_each_online_cpu (cpu) {
		for (event = 0; event < N_HTMMEVENTS; event++) {
			if (mem_event[cpu][event])
				perf_event_disable(mem_event[cpu][event]);
		}
	}
}

static void pebs_enable(void)
{
    int cpu, event;

    printk("pebs enable\n");
    for_each_online_cpu (cpu) {
	for (event = 0; event < N_HTMMEVENTS; event++) {
	    if (mem_event[cpu][event])
		perf_event_enable(mem_event[cpu][event]);
	}
    }
}

static void pebs_update_period(uint64_t value, uint64_t inst_value)
{
    int cpu, event;

    for_each_online_cpu (cpu) {
	for (event = 0; event < N_HTMMEVENTS; event++) {
	    int ret;
	    if (!mem_event[cpu][event])
		continue;

	    switch (event) {
		case DRAMREAD:
		case NVMREAD:
		case CXLREAD:
		    ret = perf_event_period(mem_event[cpu][event], value);
		    break;
		case MEMWRITE:
		    ret = perf_event_period(mem_event[cpu][event], inst_value);
		    break;
		default:
		    ret = 0;
		    break;
	    }

	    if (ret == -EINVAL)
		printk("failed to update sample period");
	}
    }
}

#define mapia_last_region(t) (container_of(t->regions_list.prev, struct mapia_region, list))

#define mapia_next_region(r) (container_of(r->list.next, struct mapia_region, list))

#define mapia_prev_region(r) (container_of(r->list.prev, struct mapia_region, list))

#define mapia_for_each_region(r, t) list_for_each_entry(r, &t->regions_list, list)

#define mapia_for_each_region_safe(r, next, t) list_for_each_entry_safe(r, next, &t->regions_list, list)

#define mapia_for_each_to_split_region(r, t) list_for_each_entry(r, &t->regions_to_split, to_split_list)

#define mapia_for_each_to_split_region_safe(r, next, t) \
    list_for_each_entry_safe(r, next, &t->regions_to_split, to_split_list)

#define mapia_for_each_task(t) list_for_each_entry(t, &tasks_list, list)

#define mapia_for_each_task_safe(t, next) list_for_each_entry_safe(t, next, &tasks_list, list)

#define log(fmt, ...) pr_info("[mapia_event]: " fmt, ##__VA_ARGS__)

static DECLARE_WAIT_QUEUE_HEAD(mapia_wait);

enum TREND_TYPE { TREND_STABLE = 0, TREND_UPWARD, TREND_DOWNWARD };
// 低于200MB，需要demotion
#define PERF_LOW_WM (200UL * 1024 * 1024)
// 高于400MB，通知demotion
#define PERF_HIGH_WM (400UL * 1024 * 1024)
// 10s静默期，10s内不允许反复迁移页面
#define MIG_COOL_NS (10ULL * NSEC_PER_SEC)
// 每轮最多迁移10K页
#define MAX_MIG_PAGES_PER_ROUND (10 * 1024)

enum hot_state {
    COLD = 0,
    WARM,
    HOT,
};
struct mapia_region {
    /* 基础字段 */
    unsigned long vm_start;
    unsigned long vm_end;
    unsigned long sampling_addr;
    unsigned int nr_accesses;

    /* 动态感知字段 */
    unsigned int last_access; // 上次聚合周期访问次数
    unsigned long access_history[5]; // 环形缓冲区存储最近5次访问
    u32 history_idx; // 当前写入位置

    // 避免小数，都用千分比表示
    /* 趋势分析字段 */
    u32 trend_strength; // 趋势强度 0~1000
    /* EWMA相关 */
    u32 ewma_change_rate; // 指数加权移动平均变化率，0~1000
    u16 alpha; // 动态平滑因子
    enum TREND_TYPE trend; // 趋势类型（增长/下降/平稳）

    /* 冷却控制 */
    u64 last_adjust_time; // 最后一次操作的时间戳

    struct list_head to_split_list;

    struct list_head list;

    // region的状态
    enum hot_state state;
    // 上次迁移时间
    u64 last_mig_time;
};
//
struct mapia_task {
    unsigned long pid;
    unsigned int changed_regions_cnt;
    struct list_head regions_to_split;
    struct list_head regions_list; /* list of mapia_region objects */
    struct list_head list; /* head for tasks list */
    spinlock_t lock;
};

/* List of mapia_task objects */
static LIST_HEAD(tasks_list);

// Promotion 迁移目标大小 (默认 200MB)
static unsigned long promotion_size = 200 * 1024 * 1024;
// 性能层最小空闲内存阈值 (默认 200MB)
static unsigned long perf_min_free = 200 * 1024 * 1024;

/* Nano-seconds */
#define M_SECOND 1000UL * 1000UL
// 采样间隔
static unsigned long min_sample_interval = 100 * M_SECOND;
static unsigned long sample_interval = 500 * M_SECOND;
// 聚合间隔，决定了汇总采样数据的频率 10秒
static unsigned long aggr_interval = 10000 * M_SECOND;
// 区域更新间隔:2s
static unsigned long regions_update_interval = 20000 * M_SECOND;

// 上次聚合时间
static struct timespec64 last_aggregate_time;
// 上次区域更新时间
static struct timespec64 last_regions_update_time;

// 最小区域数
static unsigned long min_nr_regions = 10;
// 最大区域数
static unsigned long max_nr_regions = 1000;

// 是否开启采样
bool mapia_tracing_on;

// 合并阈值
static unsigned long mapia_merge_threshold = 100; /* 10 percent */

// 随机种子
struct rnd_state rndseed;

/* result buffer */
#define MAPIA_LEN_RBUF (1024 * 1024 * 4)
static char mapia_rbuf[MAPIA_LEN_RBUF];
static unsigned int mapia_rbuf_offset;

/* result file */
#define LEN_RES_FILE_PATH 256
/* TODO: Make this path configurable */
static char rfile_path[LEN_RES_FILE_PATH] = "/mapia.res.bin";
static struct file *rfile;

static struct task_struct *mapia_trace_task;
static bool check_local_change(struct mapia_region *r);
static void trigger_split(struct mapia_task *t, struct mapia_region *r);
static bool check_global_change(unsigned int total_regions_cnt, unsigned int changed_regions_cnt);

static inline unsigned long mapia_sampling_target(struct mapia_region *r)
{
    return r->vm_start + prandom_u32_state(&rndseed) % (r->vm_end - r->vm_start);
}

/*
 * Return new region object
 */
static struct mapia_region *mapia_new_region(unsigned long vm_start, unsigned long vm_end)
{
    struct mapia_region *ret;

    ret = kmalloc(sizeof(struct mapia_region), GFP_KERNEL);
    if (ret == NULL)
        return NULL;
    ret->vm_start = vm_start;
    ret->vm_end = vm_end;
    ret->nr_accesses = 0;
    ret->sampling_addr = mapia_sampling_target(ret);
    ret->nr_accesses = 0;
    ret->last_access = 0;
    ret->last_adjust_time = 0;
    memset(ret->access_history, 0, sizeof(ret->access_history));
    ret->history_idx = 0;
    ret->ewma_change_rate = 0;
    // 平滑因子默认0.5
    ret->alpha = 500;
    ret->state = COLD;
    ret->last_mig_time = 0;

    INIT_LIST_HEAD(&ret->to_split_list);
    INIT_LIST_HEAD(&ret->list);

    return ret;
}

static inline void __mapia_add_region(struct mapia_region *r, struct mapia_region *prev, struct mapia_region *next)
{
    __list_add(&r->list, &prev->list, &next->list);
}

static void mapia_add_region_to_split(struct mapia_region *r, struct mapia_task *t)
{
    list_add_tail(&r->to_split_list, &t->regions_to_split);
}

// remove reigon from to_split_list
static void mapia_rm_region_to_split(struct mapia_region *r)
{
    list_del(&r->to_split_list);
}

/*
 * Add a region into a task
 */
static void mapia_add_region(struct mapia_region *r, struct mapia_task *t)
{
    list_add_tail(&r->list, &t->regions_list);
}

/*
 * Remove a region from a task
 *
 * NOTE: It just remove the region from the list, does not de-allocate the
 * region
 */
static void mapia_rm_region(struct mapia_region *r)
{
    list_del(&r->list);
}

/*
 * Destroy a region
 */
static void mapia_destroy_region(struct mapia_region *r)
{
    kfree(r);
}

static struct mapia_task *mapia_new_task(unsigned long pid)
{
    struct mapia_task *t;

    t = kmalloc(sizeof(struct mapia_task), GFP_KERNEL);
    if (t == NULL)
        return NULL;
    t->pid = pid;
    t->changed_regions_cnt = 0;
    INIT_LIST_HEAD(&t->regions_to_split);
    INIT_LIST_HEAD(&t->regions_list);
    spin_lock_init(&t->lock);

    return t;
}

/* Get nth mapia region */
struct mapia_region *mapia_nth_region_of(struct mapia_task *t, unsigned int n)
{
    struct mapia_region *r;
    unsigned int i;

    i = 0;
    mapia_for_each_region(r, t)
    {
        if (i++ == n)
            return r;
    }
    return NULL;
}

static void mapia_add_task(struct mapia_task *t)
{
    list_add_tail(&t->list, &tasks_list);
}

static void mapia_rm_task(struct mapia_task *t)
{
    list_del(&t->list);
}

static void mapia_destroy_task(struct mapia_task *t)
{
    struct mapia_region *r, *next;

    mapia_for_each_region_safe(r, next, t)
    {
        mapia_rm_region(r);
        mapia_destroy_region(r);
    }
    kfree(t);
}

static long mapia_set_pids(pid_t pid, unsigned long nr_pids)
{
    unsigned long i;
    struct mapia_task *t, *next;
    bool found;

    /* Remove unselected tasks */
    mapia_for_each_task_safe(t, next)
    {
        found = false;
        for (i = 0; i < nr_pids; i++) {
            if (pid == t->pid) {
                found = true;
                break;
            }
        }
        if (found)
            continue;
        mapia_rm_task(t);
        mapia_destroy_task(t);
    }

    /* Add new tasks */
    for (i = 0; i < nr_pids; i++) {
        found = false;
        mapia_for_each_task(t)
        {
            if (t->pid == pid) {
                found = true;
                break;
            }
        }
        if (found)
            continue;
        t = mapia_new_task(pid);
        if (t == NULL) {
            pr_err("Failed to alloc mapia_task\n");
            return -ENOMEM;
        }
        mapia_add_task(t);
    }

    return 0;
}

static long mapia_set_attrs(unsigned long sample_int, unsigned long aggr_int, unsigned long regions_update_int,
                            unsigned long min_nr_reg, unsigned long max_nr_reg, char *path_to_rfile)
{
    if (strnlen(path_to_rfile, LEN_RES_FILE_PATH) >= LEN_RES_FILE_PATH) {
        pr_err("too long (>%d) result file path %s\n", LEN_RES_FILE_PATH, path_to_rfile);
        return -EINVAL;
    }
    sample_interval = sample_int;
    aggr_interval = aggr_int;
    regions_update_interval = regions_update_int;
    min_nr_regions = min_nr_reg;
    max_nr_regions = max_nr_reg;
    strncpy(rfile_path, path_to_rfile, LEN_RES_FILE_PATH);
    return 0;
}

struct region {
    unsigned long start;
    unsigned long end;
};

static unsigned long sz_region(struct region *r)
{
    return r->end - r->start;
}

static void swap_regions(struct region *r1, struct region *r2)
{
    struct region tmp;

    tmp = *r1;
    *r1 = *r2;
    *r2 = tmp;
}

/*
 * Find three regions in a list of vmas
 *
 * vma        head of the list of virtual memory areas that illustrating
 *        entire address space
 * regions    array of three 'struct region' that result will be saved
 *
 * Find three regions seperated by two biggest gaps in given address space,
 * which is constructed with given list of virtual memory areas.  Because gaps
 * between stack, memory mapped regions and heap are usually very huge, those
 * would be the two gaps in usual case.
 */
static void mapia_three_regions_in_vmas(struct vm_area_struct *vma, struct region regions[3])
{
    struct vm_area_struct *prev_vma = NULL;
    struct region gap =
                    {
                        0,
                    },
                first_gap =
                    {
                        0,
                    },
                second_gap = {
                    0,
                };
    unsigned long start = 0, end = 0;

    /* Find two biggest gaps */
    for (; vma; vma = vma->vm_next) {
        if (prev_vma == NULL) {
            start = vma->vm_start;
            prev_vma = vma;
            continue;
        }
        if (vma->vm_next == NULL)
            end = vma->vm_end;
        gap.start = prev_vma->vm_end;
        gap.end = vma->vm_start;
        /*
     * size of gaps should: gap < second_gap < first_gap.
     * We use bubble-sort like algorithm with two bubbles.
     */
        if (sz_region(&gap) > sz_region(&second_gap)) {
            swap_regions(&gap, &second_gap);
            if (sz_region(&second_gap) > sz_region(&first_gap))
                swap_regions(&second_gap, &first_gap);
        }
        prev_vma = vma;
    }

    /* Sort by address order */
    if (first_gap.start > second_gap.start)
        swap_regions(&first_gap, &second_gap);

    /* Store the result */
    regions[0].start = start;
    regions[0].end = first_gap.start;
    regions[1].start = first_gap.end;
    regions[1].end = second_gap.start;
    regions[2].start = second_gap.end;
    regions[2].end = end;
}

/*
 * Get three big regions in given task
 *
 * Returns 0 or negative error value if failed
 */
static int mapia_three_regions_of(struct mapia_task *t, struct region regions[3])
{
    struct task_struct *task;
    struct mm_struct *mm;

    rcu_read_lock();
    task = pid_task(find_vpid(t->pid), PIDTYPE_PID);
    if (!task)
        goto err;

    /* TODO: get/put mm */
    mm = task->mm;
    if (!mm) {
        rcu_read_unlock();
        return -EINVAL;
    }

    /* down_read(&mm->mmap_sem); */
    down_read(&mm->mmap_lock);
    mapia_three_regions_in_vmas(mm->mmap, regions);
    up_read(&mm->mmap_lock);
    rcu_read_unlock();

    return 0;

err:
    rcu_read_unlock();
    return -EINVAL;
}

void copy_region(struct mapia_region *src, struct mapia_region *dest)
{
    struct mapia_region *slip, *r;
    r = src;
    slip = dest;
    slip->last_access = r->last_access;
    memcpy(slip->access_history, r->access_history, sizeof(r->access_history));
    slip->last_adjust_time = r->last_adjust_time;
    slip->alpha = r->alpha;
    slip->ewma_change_rate = r->ewma_change_rate;
    slip->trend = r->trend;
    slip->trend_strength = r->trend_strength;
}

/*
 * Split a region into 'nr_slips' slips having same size
 */
/*
 * 将指定内存区域分割为多个等宽子区域
 * @r: 待分割的原始区域指针
 * @nr_slips: 分割份数（必须大于1）
 *
 * 分割原理：
 * +-------------------+        +-------+-------+-------+
 * | 原始区域           |  =>   | 子区域1|子区域2|...子区域N|
 * +-------------------+        +-------+-------+-------+
 */
static void mapia_split_region_n(struct mapia_region *r, unsigned int nr_slips)
{
    unsigned long start;
    unsigned long sz_orig, sz_slip, orig_end;
    struct mapia_region *slip = NULL, *next;

    /* 1. 计算原始区域参数 */
    orig_end = r->vm_end; // 保存原始结束地址
    sz_orig = orig_end - r->vm_start; // 计算原始区域大小
    sz_slip = PAGE_ALIGN(DIV_ROUND_UP(sz_orig, nr_slips)); // 计算每个子区域大小（页对齐）

    /* 2. 调整原始区域为第一个子区域 */
    r->vm_end = r->vm_start + sz_slip; // 修改原区域的结束地址
    next = mapia_next_region(r); // 获取原区域的下一个区域指针

    /* 3. 循环创建新子区域 */
    for (start = r->vm_end; start < orig_end; start += sz_slip) {
        /* 3.1 分配并初始化新区域 */
        slip = mapia_new_region(start, start + sz_slip);
        if (!slip) {
            pr_warn("Failed to split region at %lx\n", start);
            break; // 内存分配失败时终止分割
        }

        /* 3.2 将新区域插入链表 
         * 插入位置：当前区域(r)与下一个区域(next)之间
         * 链表结构更新：r -> slip -> next
         */
        __mapia_add_region(slip, r, next);

        /* 3.3 继承原始区域的动态参数 */
        /* copy_region(r, slip); */

        r = slip; // 移动指针到新创建的区域
    }

    /* 4. 修正最后一个子区域边界（避免尺寸误差） */
    if (slip)
        slip->vm_end = orig_end; // 确保最后一个子区域精确对齐原始结束地址

    /* 5. 重置原区域的采样地址 */
    r->sampling_addr = mapia_sampling_target(r);
}

/*
 * Initialize 'regions_list' of given task
 *
 * Usual memory map of normal processes is as below:
 *
 *   <code>
 *   <heap>
 *   <big gap>
 *   <file-backed or anonymous pages>
 *   <big gap>
 *   <stack>
 *   <vvar>
 *   <vdso>
 *   <vsyscall>
 *
 * This function seperates the virtual address space with three regions
 * seperated by the two big gaps.
 */
static void init_regions_of(struct mapia_task *t)
{
    struct mapia_region *r;
    struct region regions[3];
    int ret, i;

    ret = mapia_three_regions_of(t, regions);
    if (ret)
        return;

    /* Set the initial three regions of the task */
    for (i = 0; i < 3; i++) {
        r = mapia_new_region(regions[i].start, regions[i].end);
        mapia_add_region(r, t);
    }

    r = mapia_nth_region_of(t, 1);
    if (!r) {
        pr_err("Initialization didn't made three regions?\n");
        return;
    }
    mapia_split_region_n(r, min_nr_regions - 2);
}

/* Initialize 'mapia_region' data structures for every tasks */
static void init_regions(void)
{
    struct mapia_task *t;

    mapia_for_each_task(t) init_regions_of(t);
}

static inline bool is_pmd_huge(pmd_t pmd)
{
    return pmd_large(pmd) || pmd_trans_huge(pmd);
}

static inline void mapia_count_hit(struct page *page);

static struct page *phmm_get_page(unsigned long pfn)
{
	struct page *page = pfn_to_online_page(pfn);

	if (!page || !PageLRU(page) || !get_page_unless_zero(page))
		return NULL;

	if (unlikely(!PageLRU(page))) {
		put_page(page);
		page = NULL;
	}
	return page;
}

static void phmm_ptep_mkold(pte_t *pte, struct mm_struct *mm,
			     unsigned long addr)
{
	bool referenced = false;

    // 当pte不存在的时候，直接跳过，不要进行访问，不然出现空指针
    if (!pte_present(*pte) || unlikely(pte_special(*pte))) {
        return;
    }

	struct page *page = phmm_get_page(pte_pfn(*pte));

	if (!page)
		return;

	if (pte_young(*pte)) {
		referenced = true;
		*pte = pte_mkold(*pte);
	}

#ifdef CONFIG_MMU_NOTIFIER
	if (mmu_notifier_clear_young(mm, addr, addr + PAGE_SIZE))
		referenced = true;
#endif /* CONFIG_MMU_NOTIFIER */

	if (referenced)
		set_page_young(page);

	set_page_idle(page);
	put_page(page);
}

static void phmm_pmdp_mkold(pmd_t *pmd, struct mm_struct *mm,
			     unsigned long addr)
{
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	bool referenced = false;

    if (!pmd_present(*pmd) || unlikely(pmd_devmap(*pmd))) {
        return;
    }

	struct page *page = phmm_get_page(pmd_pfn(*pmd));

	if (!page)
		return;

	if (pmd_young(*pmd)) {
		referenced = true;
		*pmd = pmd_mkold(*pmd);
	}

#ifdef CONFIG_MMU_NOTIFIER
	if (mmu_notifier_clear_young(mm, addr,
				addr + ((1UL) << HPAGE_PMD_SHIFT)))
		referenced = true;
#endif /* CONFIG_MMU_NOTIFIER */

	if (referenced)
		set_page_young(page);

	set_page_idle(page);
	put_page(page);
#endif /* CONFIG_TRANSPARENT_HUGEPAGE */
}

static int phmm_mkold_pmd_entry(pmd_t *pmd, unsigned long addr,
		unsigned long next, struct mm_walk *walk)
{
	pte_t *pte;
	spinlock_t *ptl;

	if (pmd_huge(*pmd)) {
		ptl = pmd_lock(walk->mm, pmd);
		if (pmd_huge(*pmd)) {
			phmm_pmdp_mkold(pmd, walk->mm, addr);
			spin_unlock(ptl);
			return 0;
		}
		spin_unlock(ptl);
	}

	if (pmd_none(*pmd) || unlikely(pmd_bad(*pmd)))
		return 0;
	pte = pte_offset_map_lock(walk->mm, pmd, addr, &ptl);
	if (!pte_present(*pte))
		goto out;
	phmm_ptep_mkold(pte, walk->mm, addr);
out:
	pte_unmap_unlock(pte, ptl);
	return 0;
}

static struct mm_walk_ops phmm_mkold_ops = {
	.pmd_entry = phmm_mkold_pmd_entry,
};

static void phmm_va_mkold(struct mm_struct *mm, unsigned long addr)
{
	mmap_read_lock(mm);
	walk_page_range(mm, addr, addr + 1, &phmm_mkold_ops, NULL);
	mmap_read_unlock(mm);
}

struct phmm_young_walk_private {
	unsigned long *page_sz;
	bool young;
};

static int phmm_young_pmd_entry(pmd_t *pmd, unsigned long addr,
		unsigned long next, struct mm_walk *walk)
{
	pte_t *pte;
	spinlock_t *ptl;
	struct page *page;
	struct phmm_young_walk_private *priv = walk->private;

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	if (pmd_huge(*pmd)) {

        // 先确定pte/pmd存在
        if (!pmd_present(*pmd)|| unlikely(pmd_devmap(*pmd))) {
            return 0;
        }

		ptl = pmd_lock(walk->mm, pmd);
		if (!pmd_huge(*pmd)) {
			spin_unlock(ptl);
			goto regular_page;
		}
		page = phmm_get_page(pmd_pfn(*pmd));
		if (!page)
			goto huge_out;
		if (pmd_young(*pmd) || !page_is_idle(page) ||
					mmu_notifier_test_young(walk->mm,
						addr)) {
			*priv->page_sz = ((1UL) << HPAGE_PMD_SHIFT);
			priv->young = true;
		}
		put_page(page);
huge_out:
		spin_unlock(ptl);
		return 0;
	}

regular_page:
#endif	/* CONFIG_TRANSPARENT_HUGEPAGE */

	if (pmd_none(*pmd) || unlikely(pmd_bad(*pmd)))
		return -EINVAL;
	pte = pte_offset_map_lock(walk->mm, pmd, addr, &ptl);
	if (!pte_present(*pte))
		goto out;
	page = phmm_get_page(pte_pfn(*pte));
	if (!page)
		goto out;
	if (pte_young(*pte) || !page_is_idle(page) ||
			mmu_notifier_test_young(walk->mm, addr)) {
		*priv->page_sz = PAGE_SIZE;
		priv->young = true;
	}
	put_page(page);
out:
	pte_unmap_unlock(pte, ptl);
	return 0;
}

static struct mm_walk_ops phmm_young_ops = {
	.pmd_entry = phmm_young_pmd_entry,
};

static bool phmm_va_young(struct mm_struct *mm, unsigned long addr,
		unsigned long *page_sz)
{
	struct phmm_young_walk_private arg = {
		.page_sz = page_sz,
		.young = false,
	};

	mmap_read_lock(mm);
	walk_page_range(mm, addr, addr + 1, &phmm_young_ops, &arg);
	mmap_read_unlock(mm);
	return arg.young;
}

/*
 * Check access to given region
 *
 * t    task of given region
 * r    region to be checked
 *
 * This function checks whether given virtual address space region of given
 * task has been accessed since last check.  In detail, it uses the page table
 * entry access bit for a page in the region that randomly selected.
 */
static void check_access(struct mapia_task *t, struct mapia_region *r)
{
    unsigned long target_addr;
    struct task_struct *task;
    struct mm_struct *mm;
    pte_t *pte = NULL;
    pmd_t *pmd = NULL;
    spinlock_t *ptl;
    int ret;

	static struct mm_struct *last_mm;
	static unsigned long last_addr;
	static unsigned long last_page_sz = PAGE_SIZE;
	static bool last_accessed;

	/* If the region is in the last checked page, reuse the result */
	if (mm == last_mm && (ALIGN_DOWN(last_addr, last_page_sz) ==
				ALIGN_DOWN(r->sampling_addr, last_page_sz))) {
		if (last_accessed)
			r->nr_accesses++;
		return;
	}

    /* rcu for task */
    rcu_read_lock();
    task = pid_task(find_vpid(t->pid), PIDTYPE_PID);
    if (!task)
        return;

    /* TODO: mm should be get/put */
    mm = task->mm;
    rcu_read_unlock();
    if (!mm)
        return;

    target_addr = r->sampling_addr;

    last_accessed = phmm_va_young(mm,target_addr,&last_page_sz);

    if(last_accessed)
    {
        pr_info("捕捉到page被访问,开始将page所在的region都进行update_pginfo");
        r->nr_accesses++;
        // 遍历region内的所有page
        unsigned long addr_iter;
        /* 新增：遍历当前 region 的所有页并更新 pginfo */
        for (addr_iter = r->vm_start; addr_iter < r->vm_end; addr_iter += PAGE_SIZE) {
            /* 调用 pginfo 更新函数 */
            update_pginfo(t->pid, addr_iter, 0);
        }
    }

    last_mm = mm;
    last_addr = target_addr;

    /* mkold next target */
    r->sampling_addr = mapia_sampling_target(r);
    target_addr = r->sampling_addr;

    phmm_va_mkold(mm,target_addr);
}

/*
 * Check whether given time interval is elapsed
 *
 * See whether the time has passed since given baseline for given interval.  If
 * so, it also set the baseline as current time for later check.
 *
 * Returns true if the time has elapsed, or false.
 */
static bool mapia_check_time_interval_reset(struct timespec64 *baseline, unsigned long interval)
{
    struct timespec64 now;

    ktime_get_coarse_ts64(&now);
    if (timespec64_to_ns(&now) - timespec64_to_ns(baseline) < interval)
        return false;
    *baseline = now;
    return true;
}

/*
 * Check whether it is time to aggregate samples
 *
 * Returns true if it is, false else.
 */
static bool need_aggregate(void)
{
    return mapia_check_time_interval_reset(&last_aggregate_time, aggr_interval);
}

static unsigned int nr_mapia_tasks(void)
{
    struct mapia_task *t;
    unsigned int ret = 0;

    mapia_for_each_task(t) ret++;
    return ret;
}

/*
 * Return number of regions for given process
 *
 * TODO: Optimization?
 */
static unsigned int nr_mapia_regions(struct mapia_task *t)
{
    struct mapia_region *r;
    unsigned int ret = 0;

    mapia_for_each_region(r, t) ret++;
    return ret;
}

/*
 * Merge two adjacent regions into one region
 */
static void mapia_merge_two_regions(struct mapia_region *l, struct mapia_region *r)
{
    BUG_ON(mapia_next_region(l) != r);

    l->vm_end = r->vm_end;
    l->nr_accesses = (l->nr_accesses + r->nr_accesses) / 2;
    mapia_rm_region(r);
    mapia_destroy_region(r);
}

/*
 * merge adjacent regions having similar nr_accesses
 *
 * t        task that merge operation will make change
 * thres    threshold for similarity decision
 *
 * TODO: In case of adjacent regions having nr_accesses 0 and 1, should we
 * don't merge these regions?  Current algorithm will don't merge.  In sense of
 * human eye, those should be merged!
 */
static void mapia_merge_regions_of(struct mapia_task *t, unsigned int thres)
{
    struct mapia_region *r, *prev = NULL, *next;
    unsigned long diff, avg;

    mapia_for_each_region_safe(r, next, t)
    {
        if (prev == NULL)
            goto next;
        if (prev->vm_end != r->vm_start)
            goto next;
        avg = (prev->nr_accesses + r->nr_accesses) / 2;
        /* If average is zero, two nr_accesses are zero */
        if (!avg)
            goto merge;
        diff = r->nr_accesses > avg ? r->nr_accesses - avg : avg - r->nr_accesses;
        if (diff * 2 * 1000 / avg > thres)
            goto next;
    merge:
        mapia_merge_two_regions(prev, r);
        continue;
    next:
        prev = r;
    }
}

static void mapia_merge_regions(void)
{
    struct mapia_task *t;
    unsigned int nr_regions = 0;

    mapia_for_each_task(t) nr_regions += nr_mapia_regions(t);
    if (nr_regions < min_nr_regions * 2)
        return;

    mapia_for_each_task(t) mapia_merge_regions_of(t, mapia_merge_threshold);
}

/*
 * Split a region into two regions
 *
 * sz_r    size of the first splitted region
 */
static void mapia_split_region_at(struct mapia_region *r, unsigned long sz_r)
{
    struct mapia_region *new, *next;

    // 新区域继承原区域的访问计数
    new = mapia_new_region(r->vm_start + sz_r, r->vm_end);
    new->nr_accesses = r->nr_accesses;
    new->last_access = r->last_access;
    memcpy(new->access_history, r->access_history, sizeof(r->access_history));
    new->history_idx = r->history_idx;

    r->vm_end = new->vm_start;
    r->sampling_addr = mapia_sampling_target(r);

    next = mapia_next_region(r);
    __mapia_add_region(new, r, next);
}

static void mapia_split_regions_of(struct mapia_task *t)
{
    struct mapia_region *r, *next;
    unsigned long nr_pages_region, nr_pages_left_region;
    unsigned long sz_left_region;

    mapia_for_each_region_safe(r, next, t)
    {
        nr_pages_region = (r->vm_end - r->vm_start) / PAGE_SIZE;
        if (nr_pages_region == 1)
            continue;
        nr_pages_left_region = prandom_u32_state(&rndseed) % (nr_pages_region - 1) + 1;
        sz_left_region = nr_pages_left_region * PAGE_SIZE;

        mapia_split_region_at(r, sz_left_region);
    }
}

static void mapia_split_regions(void)
{
    struct mapia_task *t;
    unsigned int nr_regions = 0;

    mapia_for_each_task(t) nr_regions += nr_mapia_regions(t);
    if (nr_regions > max_nr_regions / 2)
        return;

    mapia_for_each_task(t) mapia_split_regions_of(t);
}

static atomic64_t perf_hits = ATOMIC64_INIT(0);
static atomic64_t capa_hits = ATOMIC64_INIT(0);
static atomic64_t total_perf_hits = ATOMIC64_INIT(0);
static atomic64_t total_capa_hits = ATOMIC64_INIT(0);

static inline void mapia_count_hit(struct page *page)
{
    if (!page) {
        return;
    }
    if (numa_has_cpu(page_to_nid(page))) {
        atomic64_inc(&perf_hits);
    } else {
        atomic64_inc(&capa_hits);
    }
}

static atomic64_t mig_total_pages = ATOMIC64_INIT(0);
static atomic64_t mig_promo_pages = ATOMIC64_INIT(0);
static atomic64_t mig_demo_pages = ATOMIC64_INIT(0);

static inline void mapia_record_mig(long pages, bool promotion)
{
    if (pages <= 0) {
        return;
    }
    atomic64_add(pages, &mig_total_pages);
    if (promotion) {
        atomic64_add(pages, &mig_promo_pages);
    } else {
        atomic64_add(pages, &mig_demo_pages);
    }
}

/*
 * 数据聚合函数
 *
 * 该函数的主要步骤为：
 * 1. 将当前时间戳和所有任务的region访存计数信息写入结果缓冲区
 * 2. 统计所有task的region总数，构造hotness_info数组
 * 3. 按照nr_accesses对所有region进行排序，选出累计不超过200MB的最热区域并打印日志
 * 4. 遍历所有任务的region，更新历史访问次数，检测变化，必要时标记切分
 * 5. 根据全局变化情况调整采样周期
 * 6. 将所有region的nr_accesses计数归零，准备进入下一轮采样
 *
 * 注意：
 * - 这里不直接调整区域划分，而是在后续流程中异步执行split/merge
 * - 通过滑动窗口和指数加权均值（EWMA）计算变化率，辅助动态自适应
 */
static void aggregate(void)
{
    struct timespec64 now;
    struct mapia_task *t;
    struct mapia_region *r, *next_r;
    unsigned int total_regions = 0, idx = 0;
    unsigned long sum_hot_size = 0;

    /* 2. 统计所有region数量 */
    mapia_for_each_task(t)
    {
        total_regions += nr_mapia_regions(t);
    }

    /* 6. 遍历所有任务，更新历史，检测局部/全局变化，必要时触发切分 */
    mapia_for_each_task(t)
    {
        unsigned int nr_regions = 0;
        mapia_for_each_region(r, t)
        {
            nr_regions++;
#ifdef DEV_1
            /* 更新最近5次的访问历史 */
            r->access_history[r->history_idx % 5] = r->nr_accesses;
            r->history_idx = (r->history_idx + 1) % 5;

            /* 检查单个region变化，若超过阈值则标记为待切分 */
            if (check_local_change(r)) {
                // 程序健壮性考虑，避免重复加入
                if (list_empty(&r->to_split_list)) {
                    mapia_add_region_to_split(r, t);
                    t->changed_regions_cnt++;
                }
            }
#endif
            /* 记录本次访问数，供下一次比较 */
            r->last_access = r->nr_accesses;
        }

#ifdef DEV_1
        /* 针对变化较大的region，执行切分操作 */
        if (t->changed_regions_cnt > 0) {
            mapia_for_each_to_split_region_safe(r, next_r, t)
            {
                trigger_split(t, r);
                mapia_rm_region_to_split(r);
            }
        }

        /* 若某个任务中变化的region比例超过阈值，调整采样参数以更敏感响应 */
        if (check_global_change(nr_regions, t->changed_regions_cnt)) {
            log("有太多区域的访存情况在发生变化:total_regions_cnt:%u,changed_regions_cnt:%u", nr_regions,
                t->changed_regions_cnt);
            sample_interval = (sample_interval - 10 * M_SECOND > min_sample_interval) ?
                                      sample_interval - 10 * M_SECOND :
                                      min_sample_interval;
            /* regions_update_interval = 100 * aggr_interval; */
        }
        t->changed_regions_cnt = 0;
#endif

        /* 7. 清零所有region的访问计数，准备下一轮采样 */
        mapia_for_each_region(r, t)
        {
            r->nr_accesses = 0;
        }
    }
}

/*
 * Check whether regions need to be updated
 *
 * Returns 1 if it is, 0 else.
 */
static int need_update_regions(void)
{
    return mapia_check_time_interval_reset(&last_regions_update_time, regions_update_interval);
}

static bool mapia_intersect(struct mapia_region *r, struct region *re)
{
    if (r->vm_end <= re->start || r->vm_start >= re->end)
        return false;
    return true;
}

static bool mapia_intersect_three(struct mapia_region *r, struct region regions[3])
{
    int i;

    if (r->vm_end <= regions[0].start || regions[2].end <= r->vm_start)
        return false;

    for (i = 0; i < 2; i++) {
        if (regions[i].end <= r->vm_start && r->vm_end <= regions[i + 1].start)
            return false;
    }
    return true;
}

static void mapia_update_two_gaps(struct mapia_task *t, struct region regions[3])
{
    struct mapia_region *r, *next;
    unsigned int i = 0;

    mapia_for_each_region_safe(r, next, t)
    {
        if (!mapia_intersect_three(r, regions)) {
            mapia_rm_region(r);
            mapia_destroy_region(r);
        }
    }

    for (i = 0; i < 3; i++) {
        struct mapia_region *first = NULL, *last;
        struct mapia_region *newr;
        struct region *re;

        re = &regions[i];
        mapia_for_each_region(r, t)
        {
            if (mapia_intersect(r, re)) {
                if (!first)
                    first = r;
                last = r;
            }
            if (r->vm_start >= re->end)
                break;
        }
        if (!first) {
            newr = mapia_new_region(re->start, re->end);
            __mapia_add_region(newr, mapia_prev_region(r), r);
        } else {
            first->vm_start = re->start;
            last->vm_end = re->end;
        }
    }
}

/*
 * Update regions with merging / splitting if necessary
 */
static void update_regions(void)
{
    struct region three_regions[3];
    struct mapia_task *t;

    mapia_for_each_task(t)
    {
        mapia_three_regions_of(t, three_regions);
        mapia_update_two_gaps(t, three_regions);
    }
}

#define pid_active(task) (task->exit_state != EXIT_ZOMBIE && task->exit_state != EXIT_DEAD)

/* 新增枚举定义 */

enum SYSTEM_MODE { STABLE_MODE = 0, VOLATILE_MODE, CRITICAL_MODE };

#define FIXED_SCALE 1000 // 定点数缩放因子

// 单个区域的变化率阈值
static u32 CHANGE_RATE_THRESHOLD = 300; // 0.5 * 1000
// 全局区域变化率阈值
static u32 TREND_THRESHOLD = 500;
// 冷却周期：1ms
static u64 COOLING_PERIOD_NS = 1 * 1000; // 1微秒
// 检测指定区域的变化情况
// return:
// true:发生了较大变化
// false：没发生大变化
static bool check_local_change(struct mapia_region *r)
{
    u64 now_ns = ktime_get_ns();
    // 冷却期检查
    u64 time_since_last = now_ns - r->last_adjust_time;
    // 没过冷却期
    if (time_since_last < COOLING_PERIOD_NS) {
        log("[time_since_last]:%lldns,still in cooling period", time_since_last);
        return false;
    }
    // 计算变化率
    u32 curr = r->nr_accesses;
    u32 last = r->last_access == 0 ? 1 : r->last_access;
    u32 change_rate = 0;
    s32 delta = abs((s32)(curr - last));
    change_rate = (delta * FIXED_SCALE) / last; // 变化率方法 FIXED_SCALE倍

    // 计算ewma变化率
    // 这里有乘法，相当于放大了 FIXED_SCALE * FIXED_SCALE倍，所以需要除以一个FIXED_SCALE
    r->ewma_change_rate = (r->alpha * change_rate + (FIXED_SCALE - r->alpha) * r->ewma_change_rate) / FIXED_SCALE;

    // 判断是否需要调整
    bool need_adjust = false;
    if (r->ewma_change_rate > CHANGE_RATE_THRESHOLD) {
        need_adjust = true;
    }

    if (need_adjust) {
        r->last_adjust_time = now_ns;
        /* log("pid:%u range:%lx-%lx ewma_change_rate:%u.%u%%", get_current()->pid, r->vm_start, r->vm_end, */
        /*     r->ewma_change_rate / 10, r->ewma_change_rate % 10); */
    }

    return need_adjust;
}

// 检查全局情况
static bool check_global_change(unsigned int total_regions_cnt, unsigned int changed_regions_cnt)
{
    if (changed_regions_cnt * 1000 / total_regions_cnt >= TREND_THRESHOLD) {
        return true;
    }
    return false;
}

// 这里应该改成异步切分，当聚合数据完毕以后才能进行切分
static void trigger_split(struct mapia_task *t, struct mapia_region *r)
{
    mapia_split_region_n(r, 2);
}

static ssize_t trend_stats_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    return simple_read_from_buffer(buf, count, ppos, "TODO\n", 5);
}

static ssize_t stability_score_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    return simple_read_from_buffer(buf, count, ppos, "0.75\n", 5);
}

/* 文件操作结构体定义 */
static const struct file_operations trend_fops = {
    .read = trend_stats_read,
};

static const struct file_operations score_fops = {
    .read = stability_score_read,
};

static int ksamplingd(void *data)
{
    unsigned long long nr_sampled = 0, nr_dram = 0, nr_nvm = 0, nr_write = 0;
    unsigned long long nr_throttled = 0, nr_lost = 0, nr_unknown = 0;
    unsigned long long nr_skip = 0;

    /* a unit of cputime: permil (1/1000) */
    u64 total_runtime, exec_runtime, cputime = 0;
    unsigned long total_cputime, elapsed_cputime, cur;
    /* used for periodic checks*/
    unsigned long cpucap_period = msecs_to_jiffies(15000); // 15s
    unsigned long sample_period = 0;
    unsigned long sample_inst_period = 0;
    /* report cpu/period stat */
    unsigned long trace_cputime, trace_period = msecs_to_jiffies(1500); // 3s
    unsigned long trace_runtime;
    /* for timeout */ 
    unsigned long sleep_timeout;

    /* for analytic purpose */
    unsigned long hr_dram = 0, hr_nvm = 0;

	struct mapia_task *t, *t_next;
    struct mapia_region *r;
	pr_info("track\n");
    init_regions();

    /* TODO implements per-CPU node ksamplingd by using pg_data_t */
    /* Currently uses a single CPU node(0) */
    const struct cpumask *cpumask = cpumask_of_node(0);
    if (!cpumask_empty(cpumask))
        do_set_cpus_allowed(access_sampling, cpumask);

    pr_info("开始进行采样");
    while (!kthread_should_stop()) {
		int cpu, event, cond = false;
		
		// 进行采样
        mapia_for_each_task(t)
        {
            mapia_for_each_region(r, t)
            {
                // 对区域进行采样
                check_access(t, r);
            }
        }

        if (need_aggregate()) {
            mapia_merge_regions();
            aggregate();
            if (need_update_regions()) {
                update_regions();
            }
            mapia_split_regions();
        }

		wait_event_interruptible_timeout(mapia_wait, kthread_should_stop(), nsecs_to_jiffies(sample_interval));
    }

    mapia_for_each_task_safe(t, t_next)
    {
        mapia_rm_task(t);
        mapia_destroy_task(t);
    }

    return 0;
}

static int ksamplingd_run(void)
{
    int err = 0;
    
    wake_up_interruptible(&mapia_wait);
    if (!access_sampling) {
	access_sampling = kthread_run(ksamplingd, NULL, "ksamplingd");
	if (IS_ERR(access_sampling)) {
	    err = PTR_ERR(access_sampling);
	    access_sampling = NULL;
	}
    }
    return err;
}


int ksamplingd_init(pid_t pid, int node)
{
    int ret;

    if (access_sampling)
	return 0;

    // ret = pebs_init(pid, node);
    // if (ret) {
	//     printk("htmm__perf_event_init failure... ERROR:%d\n", ret);
	//     return 0;
    // }

    prandom_seed_state(&rndseed, 42);
    // 将pid加入到采样列表
    mapia_set_pids(pid, 1);
    return ksamplingd_run();
}

void ksamplingd_exit(void)
{
    if (access_sampling) {
	kthread_stop(access_sampling);
	access_sampling = NULL;
    }
    // pebs_disable();
}
