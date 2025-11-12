#!/bin/bash

# 检查参数数量
if [ $# -ne 2 ]; then
    echo "用法: $0 <vmstat_before.log> <vmstat_after.log>"
    exit 1
fi

BEFORE_FILE=$1
AFTER_FILE=$2

# 检查路径中是否包含chrono、phmm或nomad
if [[ "$BEFORE_FILE" == *"chrono"* ]]; then
    USE_CHRONO_METRICS=true
    USE_PHMM_METRICS=false
    USE_NOMAD_METRICS=false
elif [[ "$BEFORE_FILE" == *"phmm"* ]]; then
    USE_CHRONO_METRICS=false
    USE_PHMM_METRICS=true
    USE_NOMAD_METRICS=false
elif [[ "$BEFORE_FILE" == *"nomad"* ]]; then
    USE_CHRONO_METRICS=false
    USE_PHMM_METRICS=false
    USE_NOMAD_METRICS=true
else
    USE_CHRONO_METRICS=false
    USE_PHMM_METRICS=false
    USE_NOMAD_METRICS=false
fi

# 检查文件是否存在
if [ ! -f "$BEFORE_FILE" ]; then
    echo "错误: 文件不存在: $BEFORE_FILE"
    exit 1
fi

if [ ! -f "$AFTER_FILE" ]; then
    echo "错误: 文件不存在: $AFTER_FILE"
    exit 1
fi

# 从文件中提取值（如果不存在则返回0）
get_value() {
    local file=$1
    local key=$2
    local value=$(grep "^$key " "$file" | awk '{print $2}')
    echo "${value:-0}"
}

# 从perfnomad部分提取值（如果不存在则返回0）
get_perfnomad_value() {
    local file=$1
    local key=$2
    # 从 === /proc/perfnomad === 分隔符之后提取值
    local value=$(awk '/^=== \/proc\/perfnomad ===$/{flag=1; next} flag && /^'"$key"' /{print $2; exit}' "$file")
    echo "${value:-0}"
}

# 读取before文件的值
if [ "$USE_CHRONO_METRICS" = true ]; then
    pgpromote_success_before=$(get_value "$BEFORE_FILE" "pgpromote_success")
    pgpromote_demoted_before=$(get_value "$BEFORE_FILE" "pgpromote_demoted")
elif [ "$USE_PHMM_METRICS" = true ]; then
    htmm_nr_promoted_before=$(get_value "$BEFORE_FILE" "htmm_nr_promoted")
    htmm_nr_demoted_before=$(get_value "$BEFORE_FILE" "htmm_nr_demoted")
    pgpromote_demoted_before=0
elif [ "$USE_NOMAD_METRICS" = true ]; then
    success_nr_before=$(get_perfnomad_value "$BEFORE_FILE" "success_nr")
    pgpromote_candidate_demoted_before=$(get_value "$BEFORE_FILE" "pgpromote_candidate_demoted")
else
    pgpromote_file_before=$(get_value "$BEFORE_FILE" "pgpromote_file")
    pgpromote_anon_before=$(get_value "$BEFORE_FILE" "pgpromote_anon")
    pgpromote_candidate_demoted_before=$(get_value "$BEFORE_FILE" "pgpromote_candidate_demoted")
fi

if [ "$USE_PHMM_METRICS" = false ] && [ "$USE_NOMAD_METRICS" = false ]; then
    pgdemote_kswapd_before=$(get_value "$BEFORE_FILE" "pgdemote_kswapd")
    pgdemote_direct_before=$(get_value "$BEFORE_FILE" "pgdemote_direct")
    pgdemote_file_before=$(get_value "$BEFORE_FILE" "pgdemote_file")
    pgdemote_anon_before=$(get_value "$BEFORE_FILE" "pgdemote_anon")
elif [ "$USE_NOMAD_METRICS" = true ]; then
    pgdemote_kswapd_before=$(get_value "$BEFORE_FILE" "pgdemote_kswapd")
    pgdemote_direct_before=$(get_value "$BEFORE_FILE" "pgdemote_direct")
    pgdemote_file_before=$(get_value "$BEFORE_FILE" "pgdemote_file")
    pgdemote_anon_before=$(get_value "$BEFORE_FILE" "pgdemote_anon")
fi

# 读取after文件的值
if [ "$USE_CHRONO_METRICS" = true ]; then
    pgpromote_success_after=$(get_value "$AFTER_FILE" "pgpromote_success")
    pgpromote_demoted_after=$(get_value "$AFTER_FILE" "pgpromote_demoted")
elif [ "$USE_PHMM_METRICS" = true ]; then
    htmm_nr_promoted_after=$(get_value "$AFTER_FILE" "htmm_nr_promoted")
    htmm_nr_demoted_after=$(get_value "$AFTER_FILE" "htmm_nr_demoted")
    pgpromote_demoted_after=0
elif [ "$USE_NOMAD_METRICS" = true ]; then
    success_nr_after=$(get_perfnomad_value "$AFTER_FILE" "success_nr")
    pgpromote_candidate_demoted_after=$(get_value "$AFTER_FILE" "pgpromote_candidate_demoted")
else
    pgpromote_file_after=$(get_value "$AFTER_FILE" "pgpromote_file")
    pgpromote_anon_after=$(get_value "$AFTER_FILE" "pgpromote_anon")
    pgpromote_candidate_demoted_after=$(get_value "$AFTER_FILE" "pgpromote_candidate_demoted")
fi

if [ "$USE_PHMM_METRICS" = false ] && [ "$USE_NOMAD_METRICS" = false ]; then
    pgdemote_kswapd_after=$(get_value "$AFTER_FILE" "pgdemote_kswapd")
    pgdemote_direct_after=$(get_value "$AFTER_FILE" "pgdemote_direct")
    pgdemote_file_after=$(get_value "$AFTER_FILE" "pgdemote_file")
    pgdemote_anon_after=$(get_value "$AFTER_FILE" "pgdemote_anon")
elif [ "$USE_NOMAD_METRICS" = true ]; then
    pgdemote_kswapd_after=$(get_value "$AFTER_FILE" "pgdemote_kswapd")
    pgdemote_direct_after=$(get_value "$AFTER_FILE" "pgdemote_direct")
    pgdemote_file_after=$(get_value "$AFTER_FILE" "pgdemote_file")
    pgdemote_anon_after=$(get_value "$AFTER_FILE" "pgdemote_anon")
fi

# 计算promote的差值
if [ "$USE_CHRONO_METRICS" = true ]; then
    promote_before=$pgpromote_success_before
    promote_after=$pgpromote_success_after
elif [ "$USE_PHMM_METRICS" = true ]; then
    promote_before=$htmm_nr_promoted_before
    promote_after=$htmm_nr_promoted_after
elif [ "$USE_NOMAD_METRICS" = true ]; then
    promote_before=$success_nr_before
    promote_after=$success_nr_after
else
    promote_before=$((pgpromote_file_before + pgpromote_anon_before))
    promote_after=$((pgpromote_file_after + pgpromote_anon_after))
fi
promote_diff=$((promote_after - promote_before))

# 计算demote的差值
if [ "$USE_PHMM_METRICS" = true ]; then
    demote_before=$htmm_nr_demoted_before
    demote_after=$htmm_nr_demoted_after
elif [ "$USE_NOMAD_METRICS" = true ]; then
    demote_before=$((pgdemote_kswapd_before + pgdemote_direct_before + pgdemote_file_before + pgdemote_anon_before))
    demote_after=$((pgdemote_kswapd_after + pgdemote_direct_after + pgdemote_file_after + pgdemote_anon_after))
else
    demote_before=$((pgdemote_kswapd_before + pgdemote_direct_before + pgdemote_file_before + pgdemote_anon_before))
    demote_after=$((pgdemote_kswapd_after + pgdemote_direct_after + pgdemote_file_after + pgdemote_anon_after))
fi
demote_diff=$((demote_after - demote_before))

# 计算demote后又被promote的差值
if [ "$USE_CHRONO_METRICS" = true ]; then
    demote_promote_before=$pgpromote_demoted_before
    demote_promote_after=$pgpromote_demoted_after
elif [ "$USE_PHMM_METRICS" = true ]; then
    demote_promote_before=0
    demote_promote_after=0
elif [ "$USE_NOMAD_METRICS" = true ]; then
    demote_promote_before=$pgpromote_candidate_demoted_before
    demote_promote_after=$pgpromote_candidate_demoted_after
else
    demote_promote_before=$pgpromote_candidate_demoted_before
    demote_promote_after=$pgpromote_candidate_demoted_after
fi
demote_promote_diff=$((demote_promote_after - demote_promote_before))

# 输出结果
echo "========================================="
echo "PG Promote 统计:"
if [ "$USE_CHRONO_METRICS" = true ]; then
    echo "  Before (success): $promote_before"
    echo "  After (success):  $promote_after"
elif [ "$USE_PHMM_METRICS" = true ]; then
    echo "  Before (htmm_nr_promoted): $promote_before"
    echo "  After (htmm_nr_promoted):  $promote_after"
elif [ "$USE_NOMAD_METRICS" = true ]; then
    echo "  Before (success_nr): $promote_before"
    echo "  After (success_nr):  $promote_after"
else
    echo "  Before (file+anon): $promote_before ($pgpromote_file_before + $pgpromote_anon_before)"
    echo "  After (file+anon):  $promote_after ($pgpromote_file_after + $pgpromote_anon_after)"
fi
echo "  差值 (After - Before): $promote_diff"
echo ""
echo "PG Demote 统计:"
if [ "$USE_PHMM_METRICS" = true ]; then
    echo "  Before (htmm_nr_demoted): $demote_before"
    echo "  After (htmm_nr_demoted):  $demote_after"
else
    echo "  Before (kswapd+direct+file+anon): $demote_before ($pgdemote_kswapd_before + $pgdemote_direct_before + $pgdemote_file_before + $pgdemote_anon_before)"
    echo "  After (kswapd+direct+file+anon):  $demote_after ($pgdemote_kswapd_after + $pgdemote_direct_after + $pgdemote_file_after + $pgdemote_anon_after)"
fi
echo "  差值 (After - Before): $demote_diff"
if [ "$USE_PHMM_METRICS" = false ]; then
    echo ""
    echo "PG Demote-Promote 统计 (被demote后又被promote):"
    if [ "$USE_CHRONO_METRICS" = true ]; then
        echo "  Before (pgpromote_demoted): $demote_promote_before"
        echo "  After (pgpromote_demoted):  $demote_promote_after"
    else
        echo "  Before (pgpromote_candidate_demoted): $demote_promote_before"
        echo "  After (pgpromote_candidate_demoted):  $demote_promote_after"
    fi
    echo "  差值 (After - Before): $demote_promote_diff"
fi
echo "========================================="

