#!/usr/bin/env python3
"""
download_logs.py — 不使用 QGC，通过 MAVLink FTP 下载飞控 SD 卡上的最新 ulog

用法：
    python3 download_logs.py [--port /dev/ttyUSB0] [--baud 115200] [--output-dir <your_path>/logs]

注意：
    - 此脚本需要独占串口，执行时会临时停止 MAVROS
    - 下载完成后会尝试重新启动 MAVROS
    - 如果重新启动失败，请手动运行 ./launch_env.sh

USER_CONFIG: The default --output-dir below is hard-coded from the original dev
environment.  Update it to match your own log directory before first use.
"""

import os
import sys
import time
import argparse
import subprocess
import re
from datetime import datetime
from pathlib import Path

# MAVLink 2.0 需要
os.environ['MAVLINK20'] = '1'


def log_info(msg):
    print(f"[INFO] {msg}")


def log_error(msg):
    print(f"[ERROR] {msg}", file=sys.stderr)


def log_ok(msg):
    print(f"[OK] {msg}")


def run_cmd(cmd, timeout=60):
    """运行 shell 命令，返回 (returncode, stdout, stderr)"""
    result = subprocess.run(
        cmd, shell=True, capture_output=True, text=True, timeout=timeout
    )
    return result.returncode, result.stdout, result.stderr


def stop_mavros():
    """停止 MAVROS 以释放串口"""
    log_info("停止 MAVROS 以释放串口...")
    ret, out, err = run_cmd("pkill -f 'mavros_node' || true")
    time.sleep(2)
    # 确认已停止
    ret, out, _ = run_cmd("pgrep -f 'mavros_node' || echo 'stopped'")
    if "stopped" in out:
        log_ok("MAVROS 已停止")
        return True
    else:
        log_error("MAVROS 未能停止，请手动停止后重试")
        return False


def restart_mavros():
    """尝试重新启动 MAVROS"""
    log_info("尝试重新启动 MAVROS...")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    launch_script = os.path.join(script_dir, "launch_env.sh")
    # 只启动 MAVROS 部分（从 launch_env.sh 中提取相关逻辑较复杂，这里提示用户手动启动）
    log_info("请手动运行: ./launch_env.sh")
    return False


def list_logs(port: str, baud: int) -> list:
    """列出飞控 SD 卡上的日志文件"""
    log_info(f"连接飞控 {port} @ {baud}，列出日志目录...")

    cmd = [
        "python3", "-m", "pymavlink.mavftp",
        "--device", port,
        "--baudrate", str(baud),
        "list", "/fs/microsd/log/"
    ]

    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=30,
            env={**os.environ, 'MAVLINK20': '1'}
        )
        if result.returncode != 0:
            log_error(f"列出日志失败: {result.stderr}")
            return []

        # 解析输出，提取目录名
        dirs = []
        for line in result.stdout.splitlines():
            line = line.strip()
            if not line:
                continue
            # mavftp list 输出格式类似：
            # D 2026-06-11
            # 或只是目录名
            parts = line.split()
            if len(parts) >= 2 and parts[0] in ('D', 'd'):
                dirs.append(parts[1])
            elif re.match(r'\d{4}-\d{2}-\d{2}', line):
                dirs.append(line)

        return dirs
    except Exception as e:
        log_error(f"列出日志异常: {e}")
        return []


def list_log_files(port: str, baud: int, date_dir: str) -> list:
    """列出某日期目录下的 ulog 文件"""
    log_info(f"列出 {date_dir} 下的日志文件...")

    cmd = [
        "python3", "-m", "pymavlink.mavftp",
        "--device", port,
        "--baudrate", str(baud),
        "list", f"/fs/microsd/log/{date_dir}/"
    ]

    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=30,
            env={**os.environ, 'MAVLINK20': '1'}
        )
        if result.returncode != 0:
            log_error(f"列出日志文件失败: {result.stderr}")
            return []

        files = []
        for line in result.stdout.splitlines():
            line = line.strip()
            if not line:
                continue
            # 提取文件名（假设行尾是文件名）
            parts = line.split()
            if len(parts) >= 2:
                fname = parts[-1]
            else:
                fname = line
            if fname.endswith('.ulg') or fname.endswith('.ulog'):
                files.append(fname)

        return sorted(files)
    except Exception as e:
        log_error(f"列出日志文件异常: {e}")
        return []


def download_log(port: str, baud: int, remote_path: str, local_path: str) -> bool:
    """下载单个日志文件"""
    log_info(f"下载 {remote_path} -> {local_path}")

    cmd = [
        "python3", "-m", "pymavlink.mavftp",
        "--device", port,
        "--baudrate", str(baud),
        "get", remote_path, local_path
    ]

    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=300,
            env={**os.environ, 'MAVLINK20': '1'}
        )
        if result.returncode != 0:
            log_error(f"下载失败: {result.stderr}")
            return False
        log_ok(f"下载完成: {local_path}")
        return True
    except Exception as e:
        log_error(f"下载异常: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="通过 MAVLink FTP 下载飞控 ulog")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="飞控串口")
    parser.add_argument("--baud", type=int, default=115200, help="波特率")
    # USER_CONFIG: change the default log directory to your own path
    parser.add_argument("--output-dir", default="~/Projects/PX4/SEU_MD_PX4/Tools/huaqiccc_optimize/logs",
                        help="本地保存目录")
    parser.add_argument("--no-restart", action="store_true",
                        help="下载完成后不尝试重启 MAVROS")
    args = parser.parse_args()

    output_dir = os.path.expanduser(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    # 检查 pymavlink 是否安装
    try:
        subprocess.run(["python3", "-m", "pymavlink.mavftp", "--help"],
                       capture_output=True, check=True)
    except Exception:
        log_error("pymavlink.mavftp 不可用，请安装: pip3 install --user pymavlink")
        return 1

    # 停止 MAVROS
    if not stop_mavros():
        return 1

    try:
        # 列出日期目录
        date_dirs = list_logs(args.port, args.baud)
        if not date_dirs:
            log_error("未找到日志目录")
            return 1

        latest_date = sorted(date_dirs)[-1]
        log_info(f"最新日志目录: {latest_date}")

        # 列出该目录下的日志文件
        log_files = list_log_files(args.port, args.baud, latest_date)
        if not log_files:
            log_error("未找到 ulog 文件")
            return 1

        latest_file = sorted(log_files)[-1]
        log_info(f"最新日志文件: {latest_file}")

        # 下载
        remote_path = f"/fs/microsd/log/{latest_date}/{latest_file}"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        local_name = f"{timestamp}_{latest_file}"
        local_path = os.path.join(output_dir, local_name)

        if download_log(args.port, args.baud, remote_path, local_path):
            # 同时保存一个 latest.ulg 软链接/副本，方便分析脚本读取
            latest_link = os.path.join(output_dir, "latest.ulg")
            if os.path.exists(latest_link) or os.path.islink(latest_link):
                os.remove(latest_link)
            os.symlink(local_path, latest_link)
            log_ok(f"已创建快捷方式: {latest_link}")
        else:
            return 1

    finally:
        if not args.no_restart:
            log_info("下载完成，请手动运行 ./launch_env.sh 重新启动 MAVROS")
        else:
            log_info("下载完成")

    return 0


if __name__ == "__main__":
    sys.exit(main())
