#!/usr/bin/env python3
"""根据 collect_bandwidth.sh 生成的 TSV 数据绘制折线图。

使用方式:

    python plot_bandwidth.py input.txt --output bandwidth.png

可通过 --show 参数在保存图片的同时弹窗展示。
"""

import argparse
import csv
import os
from typing import List, Tuple

import matplotlib.pyplot as plt


def detect_delimiter(header: str) -> str:
    """根据首行判断分隔符，默认为制表符。"""

    if "\t" in header:
        return "\t"
    if "," in header:
        return ","
    if ";" in header:
        return ";"
    return None  # 让 csv 自动识别


def load_bandwidth_data(path: str) -> Tuple[List[float], List[float], List[float]]:
    with open(path, "r", encoding="utf-8") as src:
        header = src.readline()
        if not header:
            raise ValueError("输入文件为空")

        delimiter = detect_delimiter(header)
        src.seek(0)

        reader = csv.DictReader(src, delimiter=delimiter if delimiter else None)

        required_keys = {
            "time_sec": None,
            "local_bandwidth_mbps": None,
            "remote_bandwidth_mbps": None,
        }

        normalized_rows = []
        for row in reader:
            normalized = {}
            for key in row:
                if key is None:
                    continue
                normalized[key.strip()] = row[key].strip() if row[key] is not None else ""

            if not normalized:
                continue

            for target_key in list(required_keys.keys()):
                if target_key in normalized:
                    required_keys[target_key] = target_key
                else:
                    for candidate in normalized.keys():
                        if candidate.replace(" ", "_").lower() == target_key.lower():
                            required_keys[target_key] = candidate
                            break
            normalized_rows.append(normalized)

        missing = [k for k, v in required_keys.items() if v is None]
        if missing:
            raise KeyError(f"文件缺少必要列: {', '.join(missing)}")

        times, locals_, remotes = [], [], []
        for row in normalized_rows:
            try:
                t = float(row[required_keys["time_sec"]])
                l = float(row[required_keys["local_bandwidth_mbps"]])
                r = float(row[required_keys["remote_bandwidth_mbps"]])
            except ValueError:
                # 跳过无法解析的行
                continue

            times.append(t)
            locals_.append(l)
            remotes.append(r)

        if not times:
            raise ValueError("未从文件中解析到有效数据")

        return times, locals_, remotes


def plot_bandwidth(times: List[float], locals_: List[float], remotes: List[float], output: str, show: bool) -> None:
    plt.style.use("seaborn-v0_8")

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, locals_, label="Local Bandwidth", color="tab:blue", linewidth=1.5)
    ax.plot(times, remotes, label="Remote Bandwidth", color="tab:orange", linewidth=1.5)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Bandwidth (MB/s)")
    ax.set_title("Memory Bandwidth (CPU 96-143)")
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
    ax.legend()

    fig.tight_layout()

    if output:
        plt.savefig(output, dpi=200)
        print(f"已保存图像: {output}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="读取 collect_bandwidth 输出并绘制折线图")
    parser.add_argument("input", help="collect_bandwidth.sh 输出的 txt 文件路径")
    parser.add_argument(
        "--output",
        "-o",
        help="可选，输出图像文件路径 (如 bandwidth.png)。若省略则仅展示或保存默认文件",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="绘制完成后在窗口中展示图像",
    )

    args = parser.parse_args()
    input_path = os.path.abspath(args.input)

    if not os.path.exists(input_path):
        raise FileNotFoundError(f"找不到输入文件: {input_path}")

    output_path = args.output
    if output_path:
        output_path = os.path.abspath(output_path)
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

    times, locals_, remotes = load_bandwidth_data(input_path)

    if not output_path and not args.show:
        base, _ = os.path.splitext(input_path)
        output_path = base + "_plot.png"

    plot_bandwidth(times, locals_, remotes, output_path, args.show)


if __name__ == "__main__":
    main()

