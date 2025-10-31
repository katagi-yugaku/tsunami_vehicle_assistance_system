# 追加: 先頭付近のimport群に
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
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
STATIC_ARGS = ["--nogui", "5.0", "20"]

# 結果を保存するJSONファイル名
OUTPUT_JSON_FILE = "simulation_averages.json"

script_name_with_system = "its102.map_one.simulation.runner"
script_name_with_nosystem = "its102.map_one.simulation.runner_nosystem"
NUM_RUNS = 50
early_rate_list = [0.1, 0.5, 0.9]
vehicle_interval = 5.0

# 追加集計のキー（stdoutで "KEY: <int>" 形式を想定）
ADDITIONAL_KEYS = [
    "INSIGHT_RANGE",
    "OBTAIN_INFO_LANE_CHANGE_COUNT",
    "ELAPSED_TIME_LANE_CHANGE_COUNT",
    "NORMALCY_BIAS_COUNT",
    "NEGATIVE_MAJORITY_BIAS_COUNT",
    "POSITIVE_MAJORITY_BIAS_COUNT",
]

# 関数シグネチャを変更（サフィックスを付けられるように）
def run_simulation_with_system(script_name: str, early_rate: float, log_suffix: str = ""):
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

    # ← 並列で競合しないようにサフィックスを付ける
    log_filename = f"log_{early_rate}{log_suffix}.txt"
    try:
        with open(log_filename, "w", encoding="utf-8") as f:
            f.write(f"{arrival_time_by_target_vehID_dict}\n")
            f.write(f"{changed_vehicle_num}\n")
            f.write(f"{extra_metrics}\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")

    return arrival_time_by_target_vehID_dict, changed_vehicle_num, extra_metrics

# 同様に nosystem 側にもサフィックス引数を追加
def run_simulation_with_nosystem(script_name: str, early_rate: float, log_suffix: str = ""):
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
                    extra_metrics[key] =  _parse_num(val_str)
                except Exception as e:
                    print(f"Error parsing {key}: {e}")

    log_filename = f"log_{early_rate}_nosystem{log_suffix}.txt"
    try:
        with open(log_filename, "w", encoding="utf-8") as f:
            f.write(f"{arrival_time_by_target_vehID_dict}\n")
            f.write(f"{changed_vehicle_num}\n")
            f.write(f"{extra_metrics}\n")
    except Exception as e:
        print(f"Error writing to log file: {e}")
    return arrival_time_by_target_vehID_dict, changed_vehicle_num, extra_metrics

def _parse_num(s: str):
    try:
        return int(s)
    except ValueError:
        return float(s)
    
# 並列実行のユーティリティ
def _run_once(mode: str, early_rate: float, run_index: int):
    if mode == "system":
        ret = run_simulation_with_system(script_name_with_system, early_rate, log_suffix=f"_{run_index}")
    else:
        ret = run_simulation_with_nosystem(script_name_with_nosystem, early_rate, log_suffix=f"_{run_index}")
    return (mode, early_rate, run_index, ret)

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


# __main__ の system 側ループを “並列化” 版に置き換え
if __name__ == "__main__":
    avg_arrival_by_vehID_with_system = {}
    avg_changed_vehicle_num_with_system = {}
    avg_extra_metrics_with_system = {er: {k: 0.0 for k in ADDITIONAL_KEYS} for er in early_rate_list}

    # 事前にコンテナ用ワーカー数を決める（必要に応じて環境変数で調整）
    MAX_WORKERS = int(os.environ.get("MAX_WORKERS", max(1, (os.cpu_count() or 2) - 1)))

    # --- system を並列実行 ---
    print(f"=== system 全ジョブを並列実行（workers={MAX_WORKERS}） ===")
    # 結果を格納するバッファ
    system_runs = {er: {"veh": [], "chg": [], "ext": []} for er in early_rate_list}

    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as ex:
        futures = []
        for er in early_rate_list:
            for i in range(NUM_RUNS):
                futures.append(ex.submit(_run_once, "system", er, i))

        for fut in as_completed(futures):
            mode, er, i, ret = fut.result()
            if ret is None:
                continue
            vehID_data_dict, changed_veh_num, extra_metrics = ret
            system_runs[er]["veh"].append(vehID_data_dict)
            system_runs[er]["chg"].append(changed_veh_num)
            system_runs[er]["ext"].append(extra_metrics)

    # 集計
    for er in early_rate_list:
        runvehID_arrival_time_dict_per_run_lists = system_runs[er]["veh"]
        change_veh_num_per_run_lists = system_runs[er]["chg"]
        extra_metrics_per_run_lists = system_runs[er]["ext"]

        avg_result = compute_average_arrival_times(runvehID_arrival_time_dict_per_run_lists)
        avg_arrival_by_vehID_with_system[er] = list(avg_result.values())

        if change_veh_num_per_run_lists:
            avg_changed_vehicle_num_with_system[er] = sum(change_veh_num_per_run_lists) / len(change_veh_num_per_run_lists)

        if extra_metrics_per_run_lists:
            for key in ADDITIONAL_KEYS:
                vals = [em.get(key, 0) for em in extra_metrics_per_run_lists]
                avg_extra_metrics_with_system[er][key] = sum(vals) / len(vals)

    # --- nosystem は early_rate=0.5 固定のまま並列 ---
    avg_arrival_by_vehID_with_nosystem = {}
    avg_changed_vehicle_num_with_nosystem = {}
    avg_extra_metrics_with_nosystem = {}

    er = 0.5
    print(f"=== nosystem early_rate={er} を並列実行（workers={MAX_WORKERS}） ===")
    nosys_runs = {"veh": [], "chg": [], "ext": []}

    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as ex:
        futures = [ex.submit(_run_once, "nosystem", er, i) for i in range(NUM_RUNS)]
        for fut in as_completed(futures):
            mode, er_ret, i, ret = fut.result()
            if ret is None:
                continue
            vehID_data_dict, changed_veh_num, extra_metrics = ret
            nosys_runs["veh"].append(vehID_data_dict)
            nosys_runs["chg"].append(changed_veh_num)
            nosys_runs["ext"].append(extra_metrics)

    avg_result = compute_average_arrival_times(nosys_runs["veh"])
    avg_arrival_by_vehID_with_nosystem[er] = list(avg_result.values())
    if nosys_runs["chg"]:
        avg_changed_vehicle_num_with_nosystem[er] = sum(nosys_runs["chg"]) / len(nosys_runs["chg"])

    avg_extra_metrics_with_nosystem[er] = {k: 0.0 for k in ADDITIONAL_KEYS}
    if nosys_runs["ext"]:
        for key in ADDITIONAL_KEYS:
            vals = [em.get(key, 0) for em in nosys_runs["ext"]]
            avg_extra_metrics_with_nosystem[er][key] = sum(vals) / len(vals)

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
