"""运行真实滤波测试程序导出CSV，再生成教学响应图、数值表及文件指纹。

先构建sample_filter_test并安装同目录requirements-filter-plots.txt，再提供
--program和--output-dir。输出目录由调用者指定；不连接硬件、不联网下载公式。
程序失败或数据不符会非零退出，已写文件须在成功结束后才作为完整报告使用。
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LABELS = {1: "LP1: tau=10 ms", 2: "Mean: 9 samples", 3: "LP2: fc=20 Hz", 4: "Notch: f0=20 Hz, Q=4"}


def read_rows(path: Path) -> list[dict[str, float]]:
    """读取生产程序的数值CSV，空相位保留为NaN以中断图上的连接线。"""
    with path.open(encoding="utf-8", newline="") as stream:
        return [{key: float(value) if value else math.nan for key, value in row.items()}
                for row in csv.DictReader(stream)]


def generate(program: Path, output: Path) -> None:
    """运行生产计算、核对数据，再交付同一批CSV、图和摘要。"""
    output.mkdir(parents=True, exist_ok=True)
    prefix = output / "responses"
    subprocess.run([str(program), "export", str(prefix)], check=True, timeout=30)
    step_path, frequency_path = output / "responses_step.csv", output / "responses_frequency.csv"
    step, frequency = read_rows(step_path), read_rows(frequency_path)
    if len(step) != 501 or len(frequency) != 320:
        raise ValueError("unexpected production CSV dimensions")
    for row in frequency:
        if row["kind"] not in LABELS or not math.isfinite(row["gain"]) or row["gain"] < 0:
            raise ValueError("invalid production frequency row")
        if abs(row["gain"] - row["reference_gain"]) > 2e-5:
            raise ValueError("gain differs from independent transfer function")

    # 固定字体、路径哈希和空日期，避免同一数据每次生成不同SVG标识。
    matplotlib.rcParams.update({"svg.hashsalt": "mc02-0077", "font.family": "DejaVu Sans", "font.size": 10})
    fig, axes = plt.subplots(3, 1, figsize=(9, 10))
    time = [row["time_s"] for row in step]
    axes[0].plot(time, [row["input"] for row in step], "k--", label="Input", linewidth=1)
    columns = ("lowpass1", "average", "lowpass2", "notch")
    for kind, column in enumerate(columns, 1):
        axes[0].plot(time, [row[column] for row in step], label=LABELS[kind])
        rows = [row for row in frequency if row["kind"] == kind]
        hz = [row["frequency_hz"] for row in rows]
        gain = [20 * math.log10(max(row["gain"], 1e-8)) for row in rows]
        line, = axes[1].semilogx(hz, gain, label=LABELS[kind])
        # 参考点来自解析传递函数，曲线来自生产逐样本计算，两者分别绘制。
        axes[1].semilogx(hz[::4], [20 * math.log10(max(row["reference_gain"], 1e-8))
                                 for row in rows[::4]], ".", color=line.get_color(), markersize=4)
        axes[2].semilogx(hz, [math.degrees(row["phase_rad"]) for row in rows], label=LABELS[kind])
    axes[0].set(xlabel="Time [s]", ylabel="Amplitude", xlim=(0, .3), title="Step at 50 ms; fs=1000 Hz")
    axes[1].set(xlabel="Frequency [Hz]", ylabel="Gain [dB]", ylim=(-80, 2), title="Production response; dots: independent reference")
    axes[2].set(xlabel="Frequency [Hz]", ylabel="Principal phase [degrees]", ylim=(-190, 190), title="Phase is undefined at the notch zero")
    for axis in axes:
        axis.grid(True, which="both", alpha=.25)
    axes[0].legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    svg_path = output / "responses.svg"
    fig.savefig(svg_path, metadata={"Date": None})
    # Matplotlib路径行会带尾随空格；去掉这些空格不改变SVG坐标，便于仓库空白检查。
    svg_path.write_text("\n".join(line.rstrip() for line in svg_path.read_text(encoding="utf-8").splitlines()) + "\n",
                        encoding="utf-8")
    fig.savefig(output / "responses.png", dpi=160)
    plt.close(fig)

    lines = ["# MC02-0077 响应数值表", "", "由生产sample_filter_test导出，fs=1000 Hz；参数仅用于测试，不是实车配置。",
             "", "| 滤波 | 频率Hz | 实测增益 | 参考增益 | 相位延迟ms |", "|---|---:|---:|---:|---:|"]
    for row in frequency:
        if row["frequency_hz"] in (5, 20, 100):
            delay = f'{row["phase_delay_ms"]:.3f}' if math.isfinite(row["phase_delay_ms"]) else "未定义"
            lines.append(f'| {LABELS[int(row["kind"])]} | {row["frequency_hz"]:g} | '
                         f'{row["gain"]:.6f} | {row["reference_gain"]:.6f} | {delay} |')
    lines += ["", "相位延迟按相位主值计算；陷波零点的相位不定义。它不等于多频群延迟或目标芯片调度耗时。",
              "原始数据见[频率CSV](responses_frequency.csv)和[阶跃CSV](responses_step.csv)，图见[SVG](responses.svg)及[PNG](responses.png)。", ""]
    (output / "summary.md").write_text("\n".join(lines), encoding="utf-8")
    sources = ["modules/algorithm/sample_filter.h", "modules/algorithm/sample_filter.c",
               "tests/host/sample_filter_test.c", "tools/workflow/plot_sample_filter.py",
               "tools/workflow/requirements-filter-plots.txt"]
    files = [step_path, frequency_path, output / "responses.svg", output / "responses.png", output / "summary.md"]
    manifest = {"matplotlib": matplotlib.__version__, "numpy": np.__version__,
                "source_sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in sources},
                "artifact_sha256": {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in files}}
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"generated verified response artifacts: {output}")


def main() -> None:
    """要求显式指定测试程序和输出位置，参数或子进程失败直接报告。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--program", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    generate(arguments.program.resolve(strict=True), arguments.output_dir.resolve())


if __name__ == "__main__":
    main()
