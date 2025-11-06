/*
 * 简单的内存访问负载测试程序
 * 用于测试 htmm_pebs_sampler 模块是否正常工作
 *
 * 使用方法：
 *   gcc -O2 -o test_load test_load.c -lpthread
 *   ./test_load [运行时间(秒)] [工作线程数]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/mman.h>

#define ARRAY_SIZE (1024 * 1024 * 1000)  /* 6400MB 数组 */
#define PAGE_SIZE 4096
#define CACHE_LINE_SIZE 64

static volatile int running = 1;
static int *array = NULL;
static size_t array_elems;

/* 工作线程函数：顺序访问 */
static void *worker_seq(void *arg)
{
    int tid = (long)arg;
    size_t i;
    int sum = 0;
    
    printf("Worker %d: 开始顺序访问模式\n", tid);
    
    while (running) {
        /* 顺序读 */
        for (i = 0; i < array_elems && running; i += CACHE_LINE_SIZE / sizeof(int)) {
            sum += array[i];
        }
        /* 顺序写 */
        for (i = 0; i < array_elems && running; i += CACHE_LINE_SIZE / sizeof(int)) {
            array[i] = sum + i;
        }
    }
    
    printf("Worker %d: 顺序访问完成，sum=%d\n", tid, sum);
    return NULL;
}

/* 工作线程函数：随机访问（更容易触发 LLC miss） */
static void *worker_rand(void *arg)
{
    int tid = (long)arg;
    size_t i, idx;
    int sum = 0;
    unsigned int seed = time(NULL) + tid;
    
    printf("Worker %d: 开始随机访问模式\n", tid);
    
    while (running) {
        /* 随机读 */
        for (i = 0; i < array_elems / 10 && running; i++) {
            idx = (rand_r(&seed) % (array_elems / (CACHE_LINE_SIZE / sizeof(int)))) 
                  * (CACHE_LINE_SIZE / sizeof(int));
            sum += array[idx];
        }
        /* 随机写 */
        for (i = 0; i < array_elems / 10 && running; i++) {
            idx = (rand_r(&seed) % (array_elems / (CACHE_LINE_SIZE / sizeof(int)))) 
                  * (CACHE_LINE_SIZE / sizeof(int));
            array[idx] = sum + idx;
        }
    }
    
    printf("Worker %d: 随机访问完成，sum=%d\n", tid, sum);
    return NULL;
}

/* 工作线程函数：跨页访问（触发更多 page fault 和 NUMA 访问） */
static void *worker_strided(void *arg)
{
    int tid = (long)arg;
    size_t i, stride;
    int sum = 0;
    
    stride = PAGE_SIZE / sizeof(int) * 7;  /* 跨页访问 */
    
    printf("Worker %d: 开始跨页访问模式 (stride=%zu)\n", tid, stride);
    
    while (running) {
        /* 跨页读 */
        for (i = 0; i < array_elems && running; i += stride) {
            sum += array[i];
        }
        /* 跨页写 */
        for (i = 0; i < array_elems && running; i += stride) {
            array[i] = sum + i;
        }
    }
    
    printf("Worker %d: 跨页访问完成，sum=%d\n", tid, sum);
    return NULL;
}

int main(int argc, char **argv)
{
    int duration = 10;  /* 默认运行 10 秒 */
    int num_threads = 2;
    pthread_t *threads;
    int i;
    long tid;
    
    if (argc > 1)
        duration = atoi(argv[1]);
    if (argc > 2)
        num_threads = atoi(argv[2]);
    
    printf("=== PEBS 采样器测试负载 ===\n");
    printf("PID: %d\n", getpid());
    printf("运行时间: %d 秒\n", duration);
    printf("工作线程数: %d\n", num_threads);
    printf("数组大小: %zu MB\n", (size_t)(ARRAY_SIZE / (1024 * 1024)));
    printf("\n提示: 使用以下命令加载模块并指定此进程 PID:\n");
    printf("  sudo insmod htmm_pebs_sampler.ko sample_pid=%d\n", getpid());
    printf("  或使用 per-cpu 模式: sudo insmod htmm_pebs_sampler.ko sample_pid=-1\n");
    printf("  查看日志: sudo cat /sys/kernel/debug/tracing/trace\n");
    printf("\n开始运行负载...\n\n");
    
    /* 分配大数组 */
    array_elems = ARRAY_SIZE / sizeof(int);
    array = mmap(NULL, ARRAY_SIZE, PROT_READ | PROT_WRITE,
                 MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    if (array == MAP_FAILED) {
        perror("mmap failed");
        return 1;
    }
    
    /* 初始化数组（触发 page fault） */
    printf("初始化数组...\n");
    memset(array, 0, ARRAY_SIZE);
    printf("数组初始化完成\n\n");
    
    /* 创建线程 */
    threads = malloc(sizeof(pthread_t) * num_threads);
    if (!threads) {
        perror("malloc failed");
        munmap(array, ARRAY_SIZE);
        return 1;
    }
    
    /* 启动工作线程 */
    for (i = 0; i < num_threads; i++) {
        tid = i;
        if (i % 3 == 0)
            pthread_create(&threads[i], NULL, worker_seq, (void *)tid);
        else if (i % 3 == 1)
            pthread_create(&threads[i], NULL, worker_rand, (void *)tid);
        else
            pthread_create(&threads[i], NULL, worker_strided, (void *)tid);
    }
    
    /* 运行指定时间 */
    sleep(duration);
    running = 0;
    
    /* 等待线程结束 */
    printf("\n等待线程结束...\n");
    for (i = 0; i < num_threads; i++) {
        pthread_join(threads[i], NULL);
    }
    
    free(threads);
    munmap(array, ARRAY_SIZE);
    
    printf("\n测试完成！\n");
    return 0;
}

