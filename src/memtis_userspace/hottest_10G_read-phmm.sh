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
    
    # PHMM specific settings
    echo 199 | tee /sys/kernel/mm/htmm/htmm_sample_period
    echo 100007 | tee /sys/kernel/mm/htmm/htmm_inst_sample_period
    echo 1 | tee /sys/kernel/mm/htmm/htmm_thres_hot
    echo 2 | tee /sys/kernel/mm/htmm/htmm_split_period
    echo 100000 | tee /sys/kernel/mm/htmm/htmm_adaptation_period
    echo 2000000 | tee /sys/kernel/mm/htmm/htmm_cooling_period
    echo 2 | tee /sys/kernel/mm/htmm/htmm_mode
    echo 500 | tee /sys/kernel/mm/htmm/htmm_demotion_period_in_ms
    echo 500 | tee /sys/kernel/mm/htmm/htmm_promotion_period_in_ms
    echo 4 | tee /sys/kernel/mm/htmm/htmm_gamma
    echo 30 | tee /sys/kernel/mm/htmm/ksampled_soft_cpu_quota
    echo 1 | tee /sys/kernel/mm/htmm/htmm_thres_split
}

function func_prepare() {
    echo "Preparing PHMM benchmark start..."
    # setting for cpu

	# disable automatic numa balancing for PHMM
    echo 0 | sudo tee /sys/kernel/mm/numa/demotion_enabled
    echo 0 | sudo tee /proc/sys/kernel/numa_balancing
    swapoff -a	# set configs
	func_memtis_setting

}

function func_main() {
    ${memtis_userspace}/bin/kill_ksampled #also kill ssh
    TIME="time"

    # make directory for run-memtis-10g-read/results-phmm
    mkdir -p ${result_dir}/run-memtis-10g-read/microbench/phmm
    LOG_DIR=${result_dir}/run-memtis-10g-read/microbench/phmm

    # flush cache
    func_cache_flush
    sleep 2

    cat /proc/vmstat | grep -e anon -e demote -e migrate -e promote -e file > ${LOG_DIR}/before_vmstat.log

	${TIME} -f "execution time %e (s)" \
	${memtis_userspace}/bin/launch_bench_nopid     ${BENCH_RUN}  2>&1 \
	    | tee ${LOG_DIR}/output.log

    cat /proc/vmstat | grep -e anon -e demote -e migrate -e promote -e file > ${LOG_DIR}/after_vmstat.log
    sleep 2

    sudo dmesg -c > ${LOG_DIR}/dmesg.txt
}


################################ Main ##################################

result_dir=${memtis_userspace}/results/ycsb/10G-hottest/read
mkdir -p ${result_dir}

# PHMM benchmark command - using zipfian workload for 10G hottest read
# Using memory ratio 1:2 (DRAM:NVM) for 10G hottest read workload
BENCH_RUN="${tools}/tpp_mem_access -fwarmup=${memtis_userspace}/../tmp/output/warmup_zipfan_hottest_10G.bin -frun=${memtis_userspace}/../tmp/output/run_zipfan_hottest_10G.bin -fout=${result_dir}/zipfan_hottest_10G.read.log --logtostderr -sleep=10 -work=2"

func_prepare
func_main
