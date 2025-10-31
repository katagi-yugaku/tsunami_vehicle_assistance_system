import subprocess
import re
import ast
import statistics
from collections import defaultdict
import sys
import json
import matplotlib.pyplot as plt
import numpy as np
import datetime
import re
from collections import defaultdict

_ID_RE = re.compile(r'^(?:init|newveh)_ShelterA_1_(\d+)(?:_\d+)?$')

# --- ユーザー設定 ---
# 実行するシミュレーションスクリプトのモジュール名
SIM_SCRIPTS = [
    "its102.map_one.simulation.runner",
    "its102.map_one.simulation.runner_nosystem"
]

# シミュレーションの固定引数
STATIC_ARGS = ["--nogui", "5.0", "50"]

# 結果を保存するJSONファイル名
OUTPUT_JSON_FILE = "simulation_averages.json"

script_name_with_system = "its102.map_one.simulation.runner"
script_name_with_nosystem = "its102.map_one.simulation.runner_nosystem"
NUM_RUNS = 10
early_rate_list = [0.1, 0.5, 0.9]
vehicle_interval = 5.0

# 追加集計のキー（stdoutで "KEY: <int>" 形式を想定）
ADDITIONAL_KEYS = [
    "OBTAIN_INFO_LANE_CHANFE_COUNT",
    "ELAPSED_TIME_LANE_CHANGE_COUNT",
    "NORMALCY_BIAS_COUNT",
    "NEGATIVE_MAJORITY_BIAS_COUNT",
    "POSITIVE_MAJORITY_BIAS_COUNT",
]

def run_simulation_with_system(script_name: str, early_rate: float):
    command = [
        "python3",
        "-m",
        script_name,
        STATIC_ARGS[0],
        str(early_rate),
        STATIC_ARGS[1],
        STATIC_ARGS[2]
    ]
    print(f"  実行中: {' '.join(command)}")
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=True,
            encoding='utf-8'
        )
    except subprocess.CalledProcessError as e:
        print(f"  [致命的エラー] コマンド実行に失敗しました: {' '.join(command)}", file=sys.stderr)
        print(f"  Return Code: {e.returncode}", file=sys.stderr)
        print(f"  Stderr: {e.stderr}", file=sys.stderr)
        return None
    except Exception as e:
        print(f"  [致命的エラー] パース中に予期せぬエラーが発生しました: {e}", file=sys.stderr)
        print(f"  コマンド: {' '.join(command)}", file=sys.stderr)
        return None

    arrival_time_by_target_vehID_dict = {}
    changed_vehicle_num = None  # 数値を想定（互換のため辞書表現でも可）
    extra_metrics = {k: 0 for k in ADDITIONAL_KEYS}

    for raw in result.stdout.splitlines():
        line = raw.strip()
        if "LANE_CHANGED_VEHICLE_NUM" in line:
            try:
                dict_str = line.split(":", 1)[1].strip()
                # 数値優先。ダメなら literal_eval
                try:
                    changed_vehicle_num = int(dict_str)
                except ValueError:
                    changed_vehicle_num = ast.literal_eval(dict_str)
            except Exception as e:
                print(f"Error parsing LANE_CHANGED_VEHICLE_NUM: {e}")

        if "arrival_time_by_vehID_dict" in line:
            try:
                dict_str = line.split(":", 1)[1].strip()
                arrival_time_by_target_vehID_dict = ast.literal_eval(dict_str)
            except Exception as e:
                print(f"Error parsing vehID dict: {e}")

        # 追加5項目
        for key in ADDITIONAL_KEYS:
            if line.startswith(f"{key}:"):
                try:
                    val_str = line.split(":", 1)[1].strip()
                    # 整数想定
                    extra_metrics[key] = int(val_str)
                except Exception as e:
                    print(f"Error parsing {key}: {e}")

    # ログ（上書き）
    log_filename = f"log_{early_rate}.txt"
    try:
        with open(log_filename, "w", encoding="utf-8") as f:
            f.write(f"{arrival_time_by_target_vehID_dict}\n")
            f.write(f"{changed_vehicle_num}\n")
            f.write(f"{extra_metrics}\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")

    # 既存2項目 + 追加5項目（第3返り値）
    return arrival_time_by_target_vehID_dict, changed_vehicle_num, extra_metrics

def run_simulation_with_nosystem(script_name: str, early_rate: float):
    command = [
        "python3",
        "-m",
        script_name,
        STATIC_ARGS[0],
        str(early_rate),
        STATIC_ARGS[1],
        STATIC_ARGS[2]
    ]
    print(f"  実行中: {' '.join(command)}")
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=True,
            encoding='utf-8'
        )
    except subprocess.CalledProcessError as e:
        print(f"  [致命的エラー] コマンド実行に失敗しました: {' '.join(command)}", file=sys.stderr)
        print(f"  Return Code: {e.returncode}", file=sys.stderr)
        print(f"  Stderr: {e.stderr}", file=sys.stderr)
        return None
    except Exception as e:
        print(f"  [致命的エラー] パース中に予期せぬエラーが発生しました: {e}", file=sys.stderr)
        print(f"  コマンド: {' '.join(command)}", file=sys.stderr)
        return None

    arrival_time_by_target_vehID_dict = {}
    changed_vehicle_num = None
    extra_metrics = {k: 0 for k in ADDITIONAL_KEYS}

    for raw in result.stdout.splitlines():
        line = raw.strip()
        if "LANE_CHANGED_VEHICLE_NUM" in line:
            try:
                dict_str = line.split(":", 1)[1].strip()
                try:
                    changed_vehicle_num = int(dict_str)
                except ValueError:
                    changed_vehicle_num = ast.literal_eval(dict_str)
            except Exception as e:
                print(f"Error parsing LANE_CHANGED_VEHICLE_NUM: {e}")

        if "arrival_time_by_vehID_dict" in line:
            try:
                dict_str = line.split(":", 1)[1].strip()
                arrival_time_by_target_vehID_dict = ast.literal_eval(dict_str)
            except Exception as e:
                print(f"Error parsing vehID dict: {e}")

        for key in ADDITIONAL_KEYS:
            if line.startswith(f"{key}:"):
                try:
                    val_str = line.split(":", 1)[1].strip()
                    extra_metrics[key] = int(val_str)
                except Exception as e:
                    print(f"Error parsing {key}: {e}")

    log_filename = f"log_{early_rate}_nosystem.txt"
    try:
        with open(log_filename, "w", encoding="utf-8") as f:
            f.write(f"{arrival_time_by_target_vehID_dict}\n")
            f.write(f"{changed_vehicle_num}\n")
            f.write(f"{extra_metrics}\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")
    return arrival_time_by_target_vehID_dict, changed_vehicle_num, extra_metrics

def compute_average_arrival_times(runvehID_arrival_time_dict_per_run_lists):
    """
    runvehID_arrival_time_dict_per_run_lists: 各シミュレーション(run)の
      {vehID(str): arrival_time(float)} を要素にもつリスト

    返り値: {基底ID(int): 平均到着時間(float)} をID昇順で並べたdict
    """
    times_by_base_id = defaultdict(list)

    for one_run in runvehID_arrival_time_dict_per_run_lists:
        for veh_key, t in one_run.items():
            m = _ID_RE.match(veh_key)
            if not m:
                # もし別フォーマットが混じっていたらスキップ（必要ならraiseに変更）
                continue
            base_id = int(m.group(1))  # 56や120など
            times_by_base_id[base_id].append(float(t))

    avg_by_id = {i: sum(ts)/len(ts) for i, ts in times_by_base_id.items()}
    # IDで昇順ソートして辞書化
    return dict(sorted(avg_by_id.items(), key=lambda kv: kv[0]))

def plot_cdfs(cdf_data_with_system: dict, cdf_data_with_nosystem: dict):
    plt.figure(figsize=(10, 6))

    for early_rate, all_arrival_times in sorted(cdf_data_with_system.items()):
        if not all_arrival_times:
            continue
        sorted_times = sorted(all_arrival_times)
        cdf = np.arange(1, len(sorted_times)+1) / len(sorted_times)
        plt.plot(sorted_times, cdf, label=f'early_rate={early_rate}')
    print(f"cdf_data_with_nosystem: {cdf_data_with_nosystem}")
    for early_rate, all_arrival_times in sorted(cdf_data_with_nosystem.items()):
        if not all_arrival_times:
            continue
        sorted_times = sorted(all_arrival_times)
        cdf = np.arange(1, len(sorted_times)+1) / len(sorted_times)
        print(f"cdf: {cdf}")
        plt.plot(sorted_times, cdf, label=f'nosystem early_rate={early_rate}')
    plt.xlabel("Arrival Time")
    plt.ylabel("CDF")
    plt.title("CDF of Arrival Times (aggregated over runs)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    # plt.show()
    plt.savefig("arrival_time_cdf.pdf", dpi=300)  # PDF出力

if __name__ == "__main__":
    avg_arrival_by_vehID_with_system = {}      # {early_rate: [avg_times]}
    avg_changed_vehicle_num_with_system = {}   # {early_rate: avg_changed_vehicle_num}

    # 追加5項目の平均値保存
    avg_extra_metrics_with_system = {er: {k: 0.0 for k in ADDITIONAL_KEYS} for er in early_rate_list}

    for early_rate in early_rate_list:
        print(f"=== system early_rate={early_rate} のsimulationを {NUM_RUNS} 回実行 ===")
        runvehID_arrival_time_dict_per_run_lists = []
        change_veh_num_per_run_lists = []
        extra_metrics_per_run_lists = []

        for i in range(NUM_RUNS):
            print(f"  → 実行 {i+1}/{NUM_RUNS}")
            ret = run_simulation_with_system(script_name=script_name_with_system, early_rate=early_rate)
            if ret is None:
                continue
            vehID_data_dict, changed_veh_num, extra_metrics = ret
            runvehID_arrival_time_dict_per_run_lists.append(vehID_data_dict)
            change_veh_num_per_run_lists.append(changed_veh_num)
            extra_metrics_per_run_lists.append(extra_metrics)

        avg_result = compute_average_arrival_times(runvehID_arrival_time_dict_per_run_lists)
        # print('avg_result:', avg_result)
        avg_arrival_by_vehID_with_system[early_rate] = list(avg_result.values())
        print('avg_arrival_by_vehID_with_system:', avg_arrival_by_vehID_with_system)

        # 既存ロジック準拠（数値を想定）。辞書が来る場合は各自でランナー出力を調整してください。
        avg_changed_vehicle_num_with_system[early_rate] = sum(change_veh_num_per_run_lists) / len(change_veh_num_per_run_lists)

        # 追加5項目の平均
        if extra_metrics_per_run_lists:
            for key in ADDITIONAL_KEYS:
                vals = [em.get(key, 0) for em in extra_metrics_per_run_lists]
                avg_extra_metrics_with_system[early_rate][key] = sum(vals) / len(vals)

    avg_arrival_by_vehID_with_nosystem = {}      # {early_rate: [avg_times]}
    avg_changed_vehicle_num_with_nosystem = {}   # {early_rate: avg_changed_vehicle_num}
    avg_extra_metrics_with_nosystem = {}         # {early_rate: {key: avg}}

    early_rate = 0.5
    print(f"=== nosystem early_rate={early_rate} のsimulationを {NUM_RUNS} 回実行 ===")
    runvehID_arrival_time_dict_per_run_lists = []
    change_veh_num_per_run_lists = []
    extra_metrics_per_run_lists = []

    for i in range(NUM_RUNS):
        print(f"  → 実行 {i+1}/{NUM_RUNS}")
        ret = run_simulation_with_nosystem(script_name=script_name_with_nosystem, early_rate=early_rate)
        if ret is None:
            continue
        vehID_data_dict, changed_veh_num, extra_metrics = ret
        runvehID_arrival_time_dict_per_run_lists.append(vehID_data_dict)
        change_veh_num_per_run_lists.append(changed_veh_num)
        extra_metrics_per_run_lists.append(extra_metrics)

    avg_result = compute_average_arrival_times(runvehID_arrival_time_dict_per_run_lists)
    avg_arrival_by_vehID_with_nosystem[early_rate] = list(avg_result.values())
    print('avg_arrival_by_vehID_with_nosystem:', avg_arrival_by_vehID_with_nosystem)

    # 既存ロジック準拠（数値を想定）
    avg_changed_vehicle_num_with_nosystem[early_rate] = sum(change_veh_num_per_run_lists) / len(change_veh_num_per_run_lists)

    # 追加5項目の平均
    avg_extra_metrics_with_nosystem[early_rate] = {k: 0.0 for k in ADDITIONAL_KEYS}
    if extra_metrics_per_run_lists:
        for key in ADDITIONAL_KEYS:
            vals = [em.get(key, 0) for em in extra_metrics_per_run_lists]
            avg_extra_metrics_with_nosystem[early_rate][key] = sum(vals) / len(vals)

    # ログ出力（system / nosystem ともに5項目の平均を含む）
    log_filename = f"log_tsunami.txt"
    try:
        with open(log_filename, "w", encoding="utf-8") as f:
            f.write(f"{datetime.datetime.now()} veh interval:{vehicle_interval}\n")
            f.write(f"system simulation\n")
            f.write(f"{avg_arrival_by_vehID_with_system}\n")
            f.write(f"{avg_changed_vehicle_num_with_system}\n")
            f.write(f"{avg_extra_metrics_with_system}\n")
            f.write(f"nosystem simulation\n")
            f.write(f"{avg_arrival_by_vehID_with_nosystem}\n")
            f.write(f"{avg_changed_vehicle_num_with_nosystem}\n")
            f.write(f"{avg_extra_metrics_with_nosystem}\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")

    # CDF描画
    plot_cdfs(avg_arrival_by_vehID_with_system, avg_arrival_by_vehID_with_nosystem)
