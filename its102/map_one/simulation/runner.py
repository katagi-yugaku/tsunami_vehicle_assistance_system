# =========================
# Standard library
# =========================
import copy
import os
import random
import sys
from collections import Counter, defaultdict
from math import sqrt
from pathlib import Path
import optparse
import sys
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
# =========================
# Third-party libraries
# =========================
# SUMO tools need SUMO_HOME on sys.path *before* importing sumolib/traci
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))

from sumolib import checkBinary  # noqa: E402
import traci  # noqa: E402
import matplotlib.pyplot as plt
import numpy as np
from numpy import double

# =========================
# Local / intra-package
# =========================
from ... import utilities
from ...agents.Agent import Agent
from ...agents.CustomeEdge import CustomeEdge, ConnectedEdges
from ...agents.Shelter import Shelter
from ...agents.VehicleInfo import VehicleInfo

# =========================
# Runtime config / seeds
# =========================
random.seed(318)  # 乱数シードを固定（再現性）
# random.seed()

COMMUNICATION_RANGE = 100
END_SIMULATION_TIME = 1500
VEHICLE_NUM = 0
DEPART_TIME: double = 0.0
ROUTE_NUM = 0
SPEED_ARRANGE = 1
CONGESTION_RATE = 0.3
INSIGHT_RANGE = 15 
SHOW_DEBUG_COUNT = 0
THRESHOLD_SPEED = 2.77 # 10km/h
STOPPING_TIME_IN_SHELTER = 15
EARLY_AGENT_THRESHOLD_LIST = [60, 90, 100, 130] # 早期決断者の閾値
LATE_AGENT_THRESHOLD_LIST = [180, 220, 300, 350] # 遅延決断者の閾値
VEH_START_TIME_BY_SHELTERID = {"ShelterA_1": 0, "ShelterA_2": 0} # 避難所ごとの車両の開始時間
TOTAL_VEHNUM = 150
NEW_VEHICLE_COUNT = 0
LANE_CHANGED_VEHICLE_COUNT = 0
NORMALCY_BIAS_COUNT = 0
POSITIVE_MAJORITY_BIAS_COUNT = 0
NEGATIVE_MAJORITY_BIAS_COUNT = 0
TSUNAMI_PRECURSOR_INFO_OBTAIN_TIME = 0
DECISION_EVALUATION_INTERVAL_FOR_INITIAL = 20.0
DECISION_EVALUATION_INTERVAL = 20.0
MOTIVATION_DECREASE_FROM_INACTIVE_NEIGHBORS = 100.0
MOTIVATION_INCREASE_FOLLOWING_NEIGHBORS = 150.0

# リストの初期化
custome_edge_list: list = []
shelter_list = []
vehInfo_list = []
vehID_list = []
veh_written_list = []
connected_edges_list = []
shelterA_arrival_time_list = []
shelterB_arrival_time_list = []
arrival_time_by_vehID_dict = {}
arrival_time_list = []
elapsed_time_list = []

# dictの初期化
current_route_dict = {}

def run(majority_bias_score: float):
    while traci.simulation.getTime() < END_SIMULATION_TIME:
        traci.simulationStep()
        control_vehicles(majority_bias_score=majority_bias_score)
    traci.close()
    sys.stdout.flush()


def control_vehicles(majority_bias_score: float):
    vehIDs = traci.vehicle.getIDList()
    global NEW_VEHICLE_COUNT
    global LANE_CHANGED_VEHICLE_COUNT
    global POSITIVE_MAJORITY_BIAS_COUNT
    global NEGATIVE_MAJORITY_BIAS_COUNT
    global NORMALCY_BIAS_COUNT
    # 現在の存在するrouteを確認する
    for routeID in traci.route.getIDList():
        # 現在のrouteを取得
        current_route_dict[routeID] = traci.route.getEdges(routeID)

    for current_vehID in vehIDs:
        vehInfo_by_current_vehID: VehicleInfo = utilities.find_vehInfo_by_vehID(current_vehID, vehInfo_list)
        shelter_for_current_vehID: Shelter = utilities.find_shelter_by_edgeID_connect_target_shelter(vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(), shelter_list)
        agent_by_current_vehID: Agent = utilities.find_agent_by_vehID(current_vehID, agent_list)
        current_edgeID: str = traci.vehicle.getRoadID(current_vehID)
        if not agent_by_current_vehID.get_created_time_flg():
            agent_by_current_vehID.set_created_time(traci.simulation.getTime())
            agent_by_current_vehID.set_created_time_flg(True)

        if current_edgeID == "-E13":
            utilities.init_driver_behavior(vehIDs = [current_vehID], lane_change_mode=1)
        if current_edgeID == "E0":
            traci.vehicle.changeLane(vehID=current_vehID, laneIndex=0, duration=1000)
        if current_edgeID == "E17":
            vehInfo_by_current_vehID.set_arrival_flag(True)
            vehInfo_by_current_vehID.set_decline_edge_arrival_flag(True)
        # 津波接近情報を取得後に、ピックアップ行動を取らないものとする
        if vehInfo_by_current_vehID.has_tsunami_precursor_info() and  vehInfo_by_current_vehID.get_edgeID_connect_target_shelter() == "E9":
            # 左折してしまっていたら、車両を生成させる
            if current_edgeID in ["E13", "E14", "E15", "E16"]:
                if current_edgeID in traci.vehicle.getIDList():
                    NEW_VEHICLE_COUNT = utilities.generate_new_veh(
                                                                            target_vehID=current_vehID, 
                                                                            NEW_VEHICLE_COUNT= NEW_VEHICLE_COUNT, 
                                                                            agent_list=agent_list, 
                                                                            vehInfo_list=vehInfo_list,
                                                                            vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
                                                                            agent_by_target_vehID=agent_by_current_vehID, 
                                                                            shelter_list=shelter_list,
                                                                            connected_edges_list=connected_edges_list,
                                                                            LATE_AGENT_THRESHOLD_LIST=LATE_AGENT_THRESHOLD_LIST,
                                                                            lane_change_mode=1
                                                                            )

        # 到着処理 # 到着によってparked_flagがTrue
        if traci.vehicle.isStoppedParking(current_vehID) and not vehInfo_by_current_vehID.get_parked_flag():
            handle_arrival(
                            current_vehID=current_vehID,
                            vehInfo_by_current_vehID=vehInfo_by_current_vehID,
                            agent_by_current_vehID=agent_by_current_vehID,
                            shelter_for_current_vehID=shelter_for_current_vehID,
                            shelter_list=shelter_list,
                            arrival_time_list=arrival_time_list,
                            arrival_time_by_vehID_dict=arrival_time_by_vehID_dict
                            )
        
        if not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():  # 減速処理を行う
            #避難地に接続する(直前の)道路にいる場合、減速する
            pre_edgeID_near_shelter_flag = \
                    utilities.is_pre_edgeID_near_shelter(
                                                            current_edgeID=current_edgeID, 
                                                            edgeID_near_shelter=vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(),
                                                            custome_edge_list=custome_edge_list
                                                            )
            if pre_edgeID_near_shelter_flag and not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():
                # 避難地直前のエッジに入った車両だけ減速制御
                local_density = utilities.get_local_density(vehID=current_vehID, radius=50.0)
                utilities.apply_gap_density_speed_control(
                                                            vehID=current_vehID,
                                                            local_density=local_density,
                                                            v_free=6.0,     # 自由流速度
                                                            v_min=2.0,      # 最低速度
                                                            gap_min=7.0,    # 強い減速を始めるギャップ
                                                            tau=1.8,        # ギャップ→速度変換の傾き
                                                            alpha=0.5,      # 平滑化
                                                            slow_time=1.0   # 速度変更時間
                                                            )
            else:
                # それ以外は自由流走行
                traci.vehicle.slowDown(current_vehID, 7.0, 1.0)

        if not vehInfo_by_current_vehID.get_arrival_flag(): # 未到着の車両に対して処理を実行
            # 通信可能範囲内にいる車両と通信を行う　通信可能範囲は100m設定になる
            if traci.simulation.getTime() % 10 == 0:
                around_vehIDs: list = utilities.get_around_vehIDs(target_vehID=current_vehID, custome_edge_list=custome_edge_list)
                utilities.v2v_communication(
                                            target_vehID=current_vehID, 
                                            target_vehInfo=vehInfo_by_current_vehID, 
                                            around_vehIDs=around_vehIDs,
                                            agent_list=agent_list,
                                            vehInfo_list=vehInfo_list,
                                            COMMUNICATION_RANGE=COMMUNICATION_RANGE
                                            )

                utilities.v2shelter_communication(
                                                    target_vehID=current_vehID, 
                                                    shelterID=vehInfo_by_current_vehID.get_target_shelter(),
                                                    vehInfo_list=vehInfo_list,
                                                    shelter_list=shelter_list,
                                                    COMMUNICATION_RANGE=COMMUNICATION_RANGE
                                                    )
                
                utilities.v2v_communication_about_tsunami_info(
                                                                target_vehID=current_vehID, 
                                                                target_vehInfo=vehInfo_by_current_vehID, 
                                                                around_vehIDs=around_vehIDs, 
                                                                vehInfo_list=vehInfo_list, 
                                                                COMMUNICATION_RANGE=COMMUNICATION_RANGE
                                                                )

            # 心理モデルの実装
            if current_edgeID in ["E2", "E3", "E4", "E5", "E6", "E7"] and not agent_by_current_vehID.get_evacuation_route_changed_flg():
                # 浮動小数対策（必要ならepsを使う or ステップ数で判定）
                if traci.simulation.getTime() % DECISION_EVALUATION_INTERVAL == 0:
                    elapsed_time = traci.simulation.getTime() - agent_by_current_vehID.get_created_time()
                    agent_by_current_vehID.update_calculated_motivation_value(current_time=elapsed_time)
                    # ★ この2つだけを以後ずっと使う（getterを再呼び出ししない）
                    mot = agent_by_current_vehID.get_calculated_motivation_value()
                    thr = agent_by_current_vehID.get_lane_change_decision_threshold()
                    # ★ Noneなら即スキップ。フラグは立てない（以後も再評価できるように）
                    if mot is None or thr is None:
                        continue
                    
                    # ★ 以降で getter を呼ばず、ローカル mot/thr を使う
                    if utilities.is_vehID_in_congested_edge(vehID=current_vehID, THRESHOLD_SPEED=THRESHOLD_SPEED):
                        if mot >= thr:
                            success_lane_change = utilities.lane_change_by_vehID(
                                                                                    vehID=current_vehID,
                                                                                    agent=agent_by_current_vehID,
                                                                                    vehInfo=vehInfo_by_current_vehID
                                                                                )
                            if success_lane_change:
                                # print("変更1！！！！")
                                LANE_CHANGED_VEHICLE_COUNT += 1
                                agent_by_current_vehID.set_evacuation_route_changed_flg(True)
                
                # 津波接近情報を取得した場合、避難行動を取るため、閾値を更新 閾値を超えるとレーンチェンジを実行
                if vehInfo_by_current_vehID.has_tsunami_precursor_info() and not agent_by_current_vehID.get_normalcy_lane_change_motivation_flg():
                    elapsed_time = traci.simulation.getTime() - agent_by_current_vehID.get_created_time()
                    # 現在値を更新してから、情報受領分を上乗せ
                    agent_by_current_vehID.update_calculated_motivation_value(current_time=elapsed_time)
                    current_motivation = agent_by_current_vehID.get_calculated_motivation_value()
                    inc = float(agent_by_current_vehID.get_motivation_increase_from_info_receive())
                    agent_by_current_vehID.set_calculated_motivation_value(current_motivation + inc)
                    agent_by_current_vehID.set_normalcy_lane_change_motivation_flg(True)
                    # ここからが修正ポイント：履歴の「該当 index 以降」に一括加算する
                    x_list = list(agent_by_current_vehID.get_x_elapsed_time_for_lane_change_list())
                    y_list = list(agent_by_current_vehID.get_y_motivation_value_for_lane_change_list())
                    # 先に index を決める（elapsed_time より後の先頭インデックス）
                    tail_start_idx = None
                    for idx, t in enumerate(x_list):
                        if elapsed_time < t:
                            tail_start_idx = idx
                            break
                        
                    if tail_start_idx is not None:
                        # 尾部に一括で加算（in-place）
                        for j in range(tail_start_idx, len(y_list)):
                            y_list[j] = float(y_list[j]) + inc
                        # setter には「リスト全体」を渡す（スカラー禁止）
                        agent_by_current_vehID.set_y_motivation_value_for_lane_change_list(y_list)
                        # XY 対応表も再構築
                        agent_by_current_vehID.set_lane_change_xy_dict(dict(zip(x_list, y_list)))
                    NORMALCY_BIAS_COUNT += 1
                    # 閾値判定（※仕様に合わせて < / >= を選択）
                    if agent_by_current_vehID.get_calculated_motivation_value() >= agent_by_current_vehID.get_lane_change_decision_threshold():
                        success_lane_change = utilities.lane_change_by_vehID(
                                                                                vehID=current_vehID,
                                                                                agent=agent_by_current_vehID,
                                                                                vehInfo=vehInfo_by_current_vehID
                                                                            )
                        if success_lane_change:
                            LANE_CHANGED_VEHICLE_COUNT += 1
                            # print("変更2！！！！")
                            agent_by_current_vehID.set_evacuation_route_changed_flg(True)
                    continue
                
                # 周囲の行動に同調する場合の処理
                if (int(traci.simulation.getTime()) % int(DECISION_EVALUATION_INTERVAL) == 0 
                        and not vehInfo_by_current_vehID.get_arrival_flag() 
                        and not agent_by_current_vehID.get_evacuation_route_changed_flg() 
                        and agent_by_current_vehID.get_normalcy_lane_change_motivation_flg()):
                    
                    # 周囲が避難行動を取らない場合、自身も合わせようとするため、閾値が減少
                    if not utilities.is_vehIDs_another_lane(target_vehID=current_vehID, vehInfo_list=vehInfo_list) and not agent_by_current_vehID.get_lane_minimum_motivation_value_flg():
                        elapsed_time = traci.simulation.getTime() - agent_by_current_vehID.get_created_time()
                        # 現在値を更新してから、情報受領分を上乗せ
                        agent_by_current_vehID.update_calculated_motivation_value(current_time=elapsed_time)
                        current_motivation = agent_by_current_vehID.get_calculated_motivation_value()
                        inc = float(agent_by_current_vehID.get_motivation_decrease_due_to_inactive_neighbors())
                        new_motivation = current_motivation - inc
                        # print(f"vehID:{current_vehID}, 経過時間: {elapsed_time} 現在値: {current_motivation}, 減少量: {inc}, 新値: {new_motivation} 閾値: {agent_by_current_vehID.get_lane_change_decision_threshold()}")
                        if new_motivation < agent_by_current_vehID.get_minimum_motivation_value():
                            agent_by_current_vehID.set_lane_minimum_motivation_value_flg(True)
                            agent_by_current_vehID.set_reach_lane_minimum_motivation_time(elapsed_time)

                            # --- 基底カーブ（元のシグモイド） ---
                            base_x = np.arange(0, 450, 1, dtype=float)
                            base_y = np.array([float(utilities.two_stage_sigmoid(x)) for x in base_x], dtype=float)

                            # --- 現在の履歴 ---
                            x_list = list(agent_by_current_vehID.get_x_elapsed_time_for_lane_change_list())
                            y_list = list(agent_by_current_vehID.get_y_motivation_value_for_lane_change_list())

                            # 下限到達時刻 t0 以降の開始 index
                            tail_start_idx = None
                            for idx, t in enumerate(x_list):
                                if elapsed_time <= t:    # ← <= で“現在バケット”も含める
                                    tail_start_idx = idx
                                    break

                            if tail_start_idx is not None:
                                theta_min = float(agent_by_current_vehID.get_minimum_motivation_value())
                                theta_dec = float(agent_by_current_vehID.get_lane_change_decision_threshold())

                                t0 = float(elapsed_time)
                                m0_raw = float(new_motivation)          # その時点の実値（下限より下になり得る）
                                b0 = float(np.interp(t0, base_x, base_y))

                                m0 = max(m0_raw, theta_min)             # ★ここが重要：下限でクランプ
                                delta = m0 - b0                         # 以後はこの Δ を足す

                                for j in range(tail_start_idx, len(y_list)):
                                    xj = float(x_list[j])
                                    bj = float(np.interp(xj, base_x, base_y))
                                    v  = bj + delta                     # シグモイドを Δ だけ上方シフト
                                    v  = max(theta_min, min(theta_dec, v))
                                    # 連続性の担保：t0直後で必ず非減少
                                    if xj >= t0:
                                        v = max(v, m0)
                                    y_list[j] = v

                                agent_by_current_vehID.set_y_motivation_value_for_lane_change_list(y_list)
                                agent_by_current_vehID.set_lane_change_xy_dict(dict(zip(x_list, y_list)))

                            # この tick の以降処理はスキップ（任意）
                            continue


                        agent_by_current_vehID.set_calculated_motivation_value(new_motivation)
                        x_list = list(agent_by_current_vehID.get_x_elapsed_time_for_lane_change_list())
                        y_list = list(agent_by_current_vehID.get_y_motivation_value_for_lane_change_list()) 
                        tail_start_idx = None
                        for idx, t in enumerate(x_list):
                            if elapsed_time < t:
                                tail_start_idx = idx
                                break
                            
                        if tail_start_idx is not None:
                            # 尾部に一括で加算（in-place）
                            for j in range(tail_start_idx, len(y_list)):
                                y_list[j] = float(new_motivation)
                            # setter には「リスト全体」を渡す（スカラー禁止）
                            agent_by_current_vehID.set_y_motivation_value_for_lane_change_list(y_list)
                            # XY 対応表も再構築
                            agent_by_current_vehID.set_lane_change_xy_dict(dict(zip(x_list, y_list)))
                        NEGATIVE_MAJORITY_BIAS_COUNT += 1 
                        if agent_by_current_vehID.get_calculated_motivation_value() >= agent_by_current_vehID.get_lane_change_decision_threshold():
                            success_lane_change = utilities.lane_change_by_vehID(
                                                                                    vehID=current_vehID,
                                                                                    agent=agent_by_current_vehID,
                                                                                    vehInfo=vehInfo_by_current_vehID
                                                                                )
                            if success_lane_change:
                                LANE_CHANGED_VEHICLE_COUNT += 1
                                # print("変更3！！！！")
                                agent_by_current_vehID.set_evacuation_route_changed_flg(True)
                    # 周囲が避難行動を取る場合、周囲の行動に同調するため、自身の閾値を上昇させる
                    else:
                        elapsed_time = traci.simulation.getTime() - agent_by_current_vehID.get_created_time()
                        # 現在値を更新してから、情報受領分を上乗せ
                        agent_by_current_vehID.update_calculated_motivation_value(current_time=elapsed_time)
                        current_motivation = agent_by_current_vehID.get_calculated_motivation_value()
                        inc = float(agent_by_current_vehID.get_motivation_increase_due_to_following_neighbors())
                        new_motivation = current_motivation + inc
                        agent_by_current_vehID.set_calculated_motivation_value(new_motivation)
                        POSITIVE_MAJORITY_BIAS_COUNT += 1 
                        # ここからが修正ポイント：履歴の「該当 index 以降」に一括加算する
                        x_list = list(agent_by_current_vehID.get_x_elapsed_time_for_lane_change_list())
                        y_list = list(agent_by_current_vehID.get_y_motivation_value_for_lane_change_list()) 
                        # 先に index を決める（elapsed_time より後の先頭インデックス）
                        tail_start_idx = None
                        for idx, t in enumerate(x_list):
                            if elapsed_time < t:
                                tail_start_idx = idx
                                break
                            
                        if tail_start_idx is not None:
                            # 尾部に一括で加算（in-place）
                            for j in range(tail_start_idx, len(y_list)):
                                y_list[j] = float(new_motivation)
                            # setter には「リスト全体」を渡す（スカラー禁止）
                            agent_by_current_vehID.set_y_motivation_value_for_lane_change_list(y_list)
                            # XY 対応表も再構築
                            agent_by_current_vehID.set_lane_change_xy_dict(dict(zip(x_list, y_list)))
                        if agent_by_current_vehID.get_calculated_motivation_value() >= agent_by_current_vehID.get_lane_change_decision_threshold():
                            success_lane_change = utilities.lane_change_by_vehID(
                                                                                    vehID=current_vehID,
                                                                                    agent=agent_by_current_vehID,
                                                                                    vehInfo=vehInfo_by_current_vehID
                                                                                )
                            if success_lane_change:
                                LANE_CHANGED_VEHICLE_COUNT += 1
                                # print(f"vehID: {current_vehID} 変更4！！！！　")
                                agent_by_current_vehID.set_evacuation_route_changed_flg(True)


            if current_edgeID == "E16":
                if traci.simulation.getTime() > 180 and traci.simulation.getTime() < 250:
                    vehInfo_by_current_vehID.update_tsunami_precursor_info(vehID=current_vehID, tsunami_precursor_flag=True, current_time=traci.simulation.getTime())
                NEW_VEHICLE_COUNT = utilities.generate_new_veh(
                                                                        target_vehID=current_vehID, 
                                                                        NEW_VEHICLE_COUNT= NEW_VEHICLE_COUNT, 
                                                                        agent_list=agent_list, 
                                                                        vehInfo_list=vehInfo_list,
                                                                        vehInfo_by_target_vehID=vehInfo_by_current_vehID, 
                                                                        agent_by_target_vehID=agent_by_current_vehID, 
                                                                        shelter_list=shelter_list,
                                                                        connected_edges_list=connected_edges_list,
                                                                        LATE_AGENT_THRESHOLD_LIST=LATE_AGENT_THRESHOLD_LIST,
                                                                        lane_change_mode=1
                                                                        )

def handle_arrival(current_vehID, vehInfo_by_current_vehID:VehicleInfo, agent_by_current_vehID:Agent,
                    shelter_for_current_vehID:Shelter, shelter_list,
                    arrival_time_list, arrival_time_by_vehID_dict):
    """
    避難地到着時の処理
    """
    arrival_time_list.append(traci.simulation.getTime())
    arrival_time_by_vehID_dict[f"{current_vehID}"] = traci.simulation.getTime()
    traci.vehicle.setSpeed(current_vehID, 9.0)
    # 避難地オブジェクトに登録
    shelter_for_current_vehID.add_arrival_vehID(current_vehID)
    vehInfo_by_current_vehID.set_evac_end_time(traci.simulation.getTime())
    # 近傍エッジから避難地オブジェクトを取得して避難時間を更新
    shelter: Shelter = utilities.find_shelter_by_edgeID_connect_target_shelter(
        agent_by_current_vehID.get_near_edgeID_by_target_shelter(), shelter_list
    )
    departure_time = traci.vehicle.getDeparture(current_vehID) + 100
    shelter.update_evac_time_default_dict(
        vehID=current_vehID,
        route=traci.vehicle.getRoute(current_vehID),
        evac_time=vehInfo_by_current_vehID.get_evac_end_time() - departure_time
    )
    # 到着フラグと駐車フラグを更新
    vehInfo_by_current_vehID.set_parked_flag(True)
    agent_by_current_vehID.set_arrival_time(traci.simulation.getTime())
    elapsed_time_list.append(traci.simulation.getTime() - agent_by_current_vehID.get_created_time())

def extract_category(vehID):
    if "ShelterA_1" in vehID:
        return "A1"
    elif "ShelterA_2" in vehID:
        return "A2"
    elif "ShelterB_1" in vehID:
        return "B1"
    elif "ShelterB_2" in vehID:
        return "B2"
    else:
        return "UNKNOWN"

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option(
                            "--nogui", action="store_true",
                            default=False, 
                            help="run the commandline version of sumo"
                            )
    options, args = optParser.parse_args()
    return options

if __name__ == "__main__":
    early_rate :float= float(sys.argv[2])
    vehicle_interval: float= float(sys.argv[3]) # 車両の生成間隔 7.0がベース
    majority_bias_score: float = float(sys.argv[4])  # 同調性バイアスの割合
    print(f"early_rate: {early_rate}, vehicle_interval: {vehicle_interval}, majority_bias_score: {majority_bias_score}")
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # シミュレーションの初期化
    # 避難地の情報をもとに、Shelter一覧を生成
    shelter_capacity_by_ID:dict = {"ShelterA_1": 150, "ShelterA_2": 150, "ShelterB_1": 150}
    edgeID_by_shelterID:dict = {"ShelterA_1": 'E17', "ShelterA_2": 'E17', "ShelterB_1": 'E9'}
    tmp_prob_list = [0.1, 0.9]
    for shelterID, near_edgeID in edgeID_by_shelterID.items():
        shelter_list:list = utilities.init_shelter(
                                                    shelterID=shelterID, 
                                                    shelter_capacity_by_ID=shelter_capacity_by_ID, 
                                                    near_edgeID=near_edgeID, 
                                                    shelter_list=shelter_list
                                                    )
    traci.start(
                [sumoBinary, "-c", "its102/map_one/data/one_shelter_one_departure.sumocfg", "--tripinfo-output", "tripinfo.xml"], 
                traceFile="traci_log.txt", 
                traceGetters=False
                )
    custome_edge_list:list = utilities.init_custom_edge()
    vehicle_start_edges:list = utilities.get_vehicle_start_edges(custome_edge_list=custome_edge_list)
    vehicle_end_edges:list = utilities.get_vehicle_end_edges(custome_edge_list=custome_edge_list)
    # 車両の開始エッジと終了エッジの組み合わせを辞書にする
    vehicle_end_list_by_start_edge_dict:dict = utilities.get_vehicle_end_list_by_start_edge_dict(vehicle_start_edges=vehicle_start_edges, vehicle_end_edges=vehicle_end_edges)
    # 全経路で総当たりをし、通行可能経路を取得しておく。
    connected_edges_list:list = utilities.import_connected_edges_from_json(file_path="its102/map_one/data/all_edgeIDs.json")
    nearest_end_edgeID_by_start_edgeID_dict:dict = utilities.import_start_end_edgeIDs_from_json(file_path="its102/map_one/data/start_end_edgeIDs.json")
    # 初期の車両を生成する
    print(f"shelterIDは: {edgeID_by_shelterID}, それぞれの選択確率は: {tmp_prob_list}")
    for vehicle_start_edgeID, end_edgeIDs in nearest_end_edgeID_by_start_edgeID_dict.items():
        for index in range(TOTAL_VEHNUM):
            end_edgeID:str = utilities.choose_edge_by_probability(edgeID_list=end_edgeIDs,probabilities=tmp_prob_list)
            shelterID_by_end_edgeID:str = utilities.find_shelterID_by_edgeID_by_shelterID(edgeID=end_edgeID, edgeID_by_shelterID=edgeID_by_shelterID)
            VEHICLE_NUM, ROUTE_NUM, vehID_list_by_shelter, DEPART_TIME = \
                utilities.generate_simple_init_vehID(
                                                        from_edgeID=vehicle_start_edgeID, 
                                                        to_edgeID=end_edgeID, 
                                                        shelterID=shelterID_by_end_edgeID, 
                                                        generate_interval=vehicle_interval,
                                                        generate_route_count=ROUTE_NUM, 
                                                        generate_veh_count=VEHICLE_NUM, 
                                                        depart_time=DEPART_TIME
                                                        )
            vehID_list.extend(vehID_list_by_shelter)

    # カテゴリのカウント
    categories = [extract_category(v) for v in vehID_list]
    counter = Counter(categories)

    # 合計と割合を表示
    total = sum(counter.values())
    print("出現数:", dict(counter))
    print("割合:")
    for cat in ["A1", "A2", "B1", "B2"]:
        count = counter.get(cat, 0)
        print(f"  {cat}: {count} ({count / total:.2%})")
    # 車両情報の初期化
    vehInfo_list:list = utilities.init_vehicleInfo_list(vehIDs=vehID_list, shelter_list=shelter_list)
    # Agentの初期化
    agent_list:list = utilities.init_agent_list(
                                                vehIDs=vehID_list, 
                                                edgeID_by_shelterID=edgeID_by_shelterID, 
                                                EARLY_AGENT_THRESHOLD_LIST=EARLY_AGENT_THRESHOLD_LIST, 
                                                LATE_AGENT_THRESHOLD_LIST=LATE_AGENT_THRESHOLD_LIST, 
                                                ATTR_RATE=early_rate,
                                                MOTIVATION_DECREASE_FROM_INACTIVE_NEIGHBORS=MOTIVATION_DECREASE_FROM_INACTIVE_NEIGHBORS,
                                                MOTIVATION_INCREASE_FOLLOWING_NEIGHBORS=MOTIVATION_INCREASE_FOLLOWING_NEIGHBORS
                                                )

    # ドライバーの行動の初期化
    utilities.init_driver_behavior(vehIDs = vehID_list, lane_change_mode=1)
    for vehID in traci.vehicle.getIDList():
        traci.vehicle.setMaxSpeed(vehID, 9.0)
    run(majority_bias_score=majority_bias_score)
    print("===== Simlation Result Summary =====")
    print("LANE_CHANGED_VEHICLE_NUM:", LANE_CHANGED_VEHICLE_COUNT)
    print("ROUTE_CHANGED_VEHICLE_NUM:", NEW_VEHICLE_COUNT)
    print("NORMALCY_BIAS_COUNT:", NORMALCY_BIAS_COUNT)
    print("NEGATIVE_MAJORITY_BIAS_COUNT:", NEGATIVE_MAJORITY_BIAS_COUNT)
    print("POSITIVE_MAJORITY_BIAS_COUNT:", POSITIVE_MAJORITY_BIAS_COUNT)
    print(f"arrival_time_by_vehID_dict: {arrival_time_by_vehID_dict}")
    # print(f"elapsed_time_list: {elapsed_time_list}")
    print(f"mean elapsed_time: {np.mean(elapsed_time_list)}")
    if len(arrival_time_list) == TOTAL_VEHNUM:
        print("OK all vehs arrived ")
    else:
        print(f"NG all vehs not arrived {len(arrival_time_list)}")
    
    for agent in agent_list:
        if agent.get_vehID() == "init_ShelterA_1_116":
            utilities.plot_dot(agent)