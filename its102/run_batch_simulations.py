import subprocess
import re
import ast
import statistics
from collections import defaultdict
import sys
import json
import matplotlib.pyplot as plt
import numpy as np
import ast
import datetime
from collections import defaultdict

# --- ユーザー設定 ---

# 試行する early_rate のリスト
EARLY_RATES = [0.1, 0.5, 0.9]

# 各 early_rate ごとに実行するシミュレーションの回数
NUM_RUNS = 2

# シミュレーションの固定引数
STATIC_ARGS = ["--nogui", "5.0", "50"]

# 結果を保存するJSONファイル名
OUTPUT_JSON_FILE = "simulation_averages.json"
script_name_with_system = "its102.map_one.simulation.runner"
script_name_with_nosystem = "its102.map_one.simulation.runner_nosystem"
NUM_RUNS = 2
early_rate_list = [0.1, 0.5, 0.9]
vehicle_interval = 5.0

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
    changed_vehicle_num_dict ={}

    for line in result.stdout.splitlines():
        line = line.strip()
        if "LANE_CHANGED_VEHICLE_NUM" in line:
            try:
                dict_str = line.split(":", 1)[1].strip()
                changed_vehicle_num_dict = ast.literal_eval(dict_str)
            except Exception as e:
                print(f"Error parsing vehID dict: {e}")
        if "arrival_time_by_vehID_dict" in line:
            try:
                dict_str = line.split(":", 1)[1].strip()
                arrival_time_by_target_vehID_dict = ast.literal_eval(dict_str)
            except Exception as e:
                print(f"Error parsing vehID dict: {e}")
    # ログファイルに初期化して書き込み（上書きモード）
    log_filename = f"log_{early_rate}.txt"
    try:
        with open(log_filename, "w", encoding="utf-8") as f:
            f.write(f"{arrival_time_by_target_vehID_dict}\n")
            f.write(f"{changed_vehicle_num_dict}\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")
    
    return arrival_time_by_target_vehID_dict, changed_vehicle_num_dict

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
    changed_vehicle_num_dict ={}

    for line in result.stdout.splitlines():
        line = line.strip()
        if "LANE_CHANGED_VEHICLE_NUM" in line:
            try:
                dict_str = line.split(":", 1)[1].strip()
                changed_vehicle_num_dict = ast.literal_eval(dict_str)
            except Exception as e:
                print(f"Error parsing vehID dict: {e}")
        if "arrival_time_by_vehID_dict" in line:
            try:
                dict_str = line.split(":", 1)[1].strip()
                arrival_time_by_target_vehID_dict = ast.literal_eval(dict_str)
            except Exception as e:
                print(f"Error parsing vehID dict: {e}")
    # ログファイルに初期化して書き込み（上書きモード）
    log_filename = f"log_{early_rate}_nosystem.txt"
    try:
        with open(log_filename, "w", encoding="utf-8") as f:
            f.write(f"{arrival_time_by_target_vehID_dict}\n")
            f.write(f"{changed_vehicle_num_dict}\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")
    return arrival_time_by_target_vehID_dict, changed_vehicle_num_dict

def compute_average_arrival_times(runvehID_arrival_time_dict_per_run_lists:list):
    from collections import defaultdict
    arrival_time_by_vehID = defaultdict(list)

    for runvehID_arrival_time_dict in runvehID_arrival_time_dict_per_run_lists:
        for vehID, arrival_time in runvehID_arrival_time_dict.items():
            arrival_time_by_vehID[vehID].append(arrival_time)

    def natural_sort_key(vehID: str):
        # 文字列中の数字を分割してタプル化 → "init_ShelterA_1_10" < "init_ShelterA_1_2" を正しく扱える
        return [int(text) if text.isdigit() else text for text in re.split(r'(\d+)', vehID)]

    average_by_vehID_dict = dict(
        sorted(
            {k: sum(vs) / len(vs) for k, vs in arrival_time_by_vehID.items()}.items(),
            key=lambda item: natural_sort_key(item[0])
        )
    )
    return average_by_vehID_dict

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
    print(f"test")
    plt.xlabel("Arrival Time")
    plt.ylabel("CDF")
    plt.title("CDF of Arrival Times (aggregated over runs)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    # plt.show()
    plt.savefig("arrival_time_cdf.pdf", dpi=300) # Changed from .png to .pdf

if __name__ == "__main__":
    avg_arrival_by_vehID_with_system = {}      # {early_rate: {vehID: avg_time}}
    avg_changed_vehicle_num_with_system = {}     # {early_rate: avg_changed_vehicle_num_with_system}

    for early_rate in early_rate_list:
        print(f"=== system early_rate={early_rate} のsimulationを {NUM_RUNS} 回実行 ===")
        runvehID_arrival_time_dict_per_run_lists = []
        change_veh_num_per_run_lists = []

        for i in range(NUM_RUNS):
            print(f"  → 実行 {i+1}/{NUM_RUNS}")
            vehID_data_dict, changed_veh_num = run_simulation_with_system(script_name=script_name_with_system, early_rate=early_rate)
            runvehID_arrival_time_dict_per_run_lists.append(vehID_data_dict)
            change_veh_num_per_run_lists.append(changed_veh_num)

        avg_result = compute_average_arrival_times(runvehID_arrival_time_dict_per_run_lists)
        avg_arrival_by_vehID_with_system[early_rate] = list(avg_result.values())
        avg_changed_vehicle_num_with_system[early_rate] = sum(change_veh_num_per_run_lists) / len(change_veh_num_per_run_lists)

    avg_arrival_by_vehID_with_nosystem = {}      # {early_rate: {vehID: avg_time}}
    avg_changed_vehicle_num_with_nosystem = {}     # {early_rate: avg_changed_vehicle_num_with_nosystem}

    early_rate = 0.5
    print(f"=== nosystem early_rate={early_rate} のsimulationを {NUM_RUNS} 回実行 ===")
    runvehID_arrival_time_dict_per_run_lists = []
    change_veh_num_per_run_lists = []
    for i in range(NUM_RUNS):
        print(f"  → 実行 {i+1}/{NUM_RUNS}")
        vehID_data_dict, changed_veh_num = run_simulation_with_nosystem(script_name=script_name_with_nosystem, early_rate=early_rate)
        runvehID_arrival_time_dict_per_run_lists.append(vehID_data_dict)
        change_veh_num_per_run_lists.append(changed_veh_num)
    avg_result = compute_average_arrival_times(runvehID_arrival_time_dict_per_run_lists)
    avg_arrival_by_vehID_with_nosystem[early_rate] = list(avg_result.values())
    print('avg_arrival_by_vehID_with_nosystem:', avg_arrival_by_vehID_with_nosystem)
    avg_changed_vehicle_num_with_nosystem[early_rate] = sum(change_veh_num_per_run_lists) / len(change_veh_num_per_run_lists)

    log_filename = f"log_tsunami.txt"
    try:
        with open(log_filename, "w", encoding="utf-8") as f:
            f.write(f"{datetime.datetime.now()} veh interval:{vehicle_interval}\n")
            f.write(f"system simulation\n")
            f.write(f"{avg_arrival_by_vehID_with_system}\n")
            f.write(f"{avg_changed_vehicle_num_with_system}\n")
            f.write(f"nosystem simulation\n")
            f.write(f"{avg_arrival_by_vehID_with_nosystem}\n")
            f.write(f"{avg_changed_vehicle_num_with_nosystem}\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")
    # CDF描画
    plot_cdfs(avg_arrival_by_vehID_with_system, avg_arrival_by_vehID_with_nosystem)