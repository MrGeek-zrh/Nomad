#!/bin/bash

# Export NVM_RATIO for use in DRAM size calculation
memtis_userspace=/home/iscas/hmm/Nomad/src/memtis_userspace
compiled_package_dir=/home/iscas/hmm/Nomad/src/memtis_userspace/
tools=${compiled_package_dir}/tools
conf=${compiled_package_dir}/conf/

function func_cache_flush() {
    echo 3 | sudo tee /proc/sys/vm/drop_caches
    free
    return
}

function func_memtis_setting() {
    echo "never" | tee /sys/kernel/mm/transparent_hugepage/enabled
    echo "never" | tee /sys/kernel/mm/transparent_hugepage/defrag
}

function func_prepare() {
    echo "Preparing benchmark start..."
    # setting for cpu

	# disable automatic numa balancing
    echo 0 | sudo tee /sys/kernel/mm/numa/demotion_enabled
    echo 0 | sudo tee /proc/sys/kernel/numa_balancing
    swapoff -a	# set configs
	func_memtis_setting

}

function func_main() {
    TIME="/usr/bin/time"

    # make directory for run-memtis-10g-read/results-pr
    if [ -n "${DRAM_SIZE}" ]; then
        mkdir -p ${result_dir}/run-memtis-10g-read/microbench/nomigrate/${DRAM_SIZE}
        LOG_DIR=${result_dir}/run-memtis-10g-read/microbench/nomigrate/${DRAM_SIZE}
    else
        mkdir -p ${result_dir}/run-memtis-10g-read/microbench/nomigrate
        LOG_DIR=${result_dir}/run-memtis-10g-read/microbench/nomigrate
    fi

    BENCH_RUN_CMD="${tools}/tpp_mem_access -fwarmup=${memtis_userspace}/../tmp/output/warmup_zipfan_hottest_10G.bin -frun=${memtis_userspace}/../tmp/output/run_zipfan_hottest_10G.bin -fout=${LOG_DIR}/zipfan_hottest_10G.read.log --logtostderr -sleep=10 -work=2"

    # flush cache
    func_cache_flush
    sleep 2


    cat /proc/vmstat | grep -e anon -e demote -e migrate -e promote -e file > ${LOG_DIR}/before_vmstat.log

	${TIME} -f "execution time %e (s)" \
    numactl --physcpubind=96-143 ${BENCH_RUN_CMD}  2>&1 \
	    | tee ${LOG_DIR}/output.log

    cat /proc/vmstat | grep -e anon -e demote -e migrate -e promote -e file > ${LOG_DIR}/after_vmstat.log
    sleep 2

    # 计算迁移统计并同时输出到控制台与 output.log
    CALC_SCRIPT=${memtis_userspace}/calculate_pgmetrics.sh
    if [ -x "$CALC_SCRIPT" ]; then
        {
            echo ""
            echo "========================================="
            echo "PG 迁移统计"
            "$CALC_SCRIPT" "${LOG_DIR}/before_vmstat.log" "${LOG_DIR}/after_vmstat.log"
        } | tee -a ${LOG_DIR}/output.log
    fi

    # 计算带宽平均值并同时输出到控制台与 output.log（从结果日志读取）
    CALC_BW=${memtis_userspace}/calculate_bandwidth.sh
    RESULT_LOG=${LOG_DIR}/zipfan_hottest_10G.read.log
    if [ -x "$CALC_BW" ] && [ -f "$RESULT_LOG" ]; then
        {
            echo ""
            echo "========================================="
            echo "带宽平均值"
            "$CALC_BW" "$RESULT_LOG"
        } | tee -a ${LOG_DIR}/output.log
    fi

    sudo dmesg -c > ${LOG_DIR}/dmesg.txt
}


################################ Main ##################################

result_dir=${memtis_userspace}/results/ycsb/10G-hottest/read
mkdir -p ${result_dir}
func_prepare
func_main
