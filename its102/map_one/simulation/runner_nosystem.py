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

COMMUNICATION_RANGE = 100
END_SIMULATION_TIME = 2000
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
TOTAL_VEHNUM = 200
NEW_VEHICLE_COUNT = 0
LANE_CHANGED_VEHICLE_COUNT = 0
POSITIVE_MAJORITY_BIAS_COUNT = 0
NEGATIVE_MAJORITY_BIAS_COUNT = 0
TSUNAMI_PRECURSOR_INFO_OBTAIN_TIME = 0

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
        
        if not vehInfo_by_current_vehID.get_decline_edge_arrival_flag() and not agent_by_current_vehID.get_evacuation_route_changed_flg(): # 減速処理を行うx
            # 避難地に接続する(直前の)道路にいる場合、減速する
            pre_edgeID_near_shelter_flag = \
                utilities.is_pre_edgeID_near_shelter(
                                                        current_edgeID=current_edgeID, 
                                                        edgeID_near_shelter=vehInfo_by_current_vehID.get_edgeID_connect_target_shelter(),
                                                        custome_edge_list=custome_edge_list
                                                        )
            if traci.vehicle.getLeader(current_vehID) is None or traci.vehicle.getLeader(current_vehID)[1] > 50:
                traci.vehicle.setColor(current_vehID, (50, 25, 255))
                traci.vehicle.setSpeed(current_vehID, 9.0)
                # or traci.vehicle.getLeader(current_vehID)[1] > 100
            else:
                if pre_edgeID_near_shelter_flag and not vehInfo_by_current_vehID.get_decline_edge_arrival_flag():
                    traci.vehicle.setColor(current_vehID, (0, 255, 0)) # 減速対象の車両を緑色に変更
                    # traci.vehicle.setSpeed(current_vehID, 3.0)
                    local_density = utilities.get_local_density(
                                                                vehID=current_vehID, 
                                                                radius=50
                                                                )
                    utilities.set_speed_by_local_density(
                                                            vehID=current_vehID,
                                                            local_density=local_density
                                                            )

        if not vehInfo_by_current_vehID.get_arrival_flag(): # 未到着の車両に対して処理を実行
            # 心理モデルの実装
            if current_edgeID in ["E2", "E3", "E4", "E5", "E6", "E7"] and current_edgeID != 'E17':
                if not agent_by_current_vehID.get_evacuation_route_changed_flg(): #TODO 渋滞の実装入れるか悩むねー
                    # レーン変更の意思決定箇所
                    if traci.simulation.getTime() % 4 == 0 and not agent_by_current_vehID.get_evacuation_route_changed_flg():
                        try:
                            newest_time = agent_by_current_vehID.get_time_lane_change_list()[-1]
                            agent_stress_value = agent_by_current_vehID.get_lane_change_xy_dict()[int(newest_time)]
                            if agent_by_current_vehID.get_lane_change_threshold_list()[-1] < agent_stress_value:
                                try:
                                    traci.vehicle.changeLane(vehID=current_vehID, laneIndex=1, duration=1000)
                                except Exception as e:
                                    print(f"Error changing lane for vehicle {current_vehID}: {e}")
                                utilities.init_driver_behavior(vehIDs = [current_vehID], lane_change_mode=512)
                                traci.vehicle.setParkingAreaStop(vehID=current_vehID, stopID="ShelterA_2", duration=100000)
                                traci.vehicle.setSpeed(current_vehID, 9.0)
                                vehInfo_by_current_vehID.set_target_shelter("ShelterA_2")
                                LANE_CHANGED_VEHICLE_COUNT += 1
                                agent_by_current_vehID.set_evacuation_route_changed_flg(True)
                                traci.vehicle.setColor(current_vehID, (255, 0, 0)) # レーン変更した車両を赤色に変更
                        except IndexError:
                            pass

                    # 正常性バイアスの実装箇所情報を受け取ると閾値が急激に減少　レーンが変更しやすくなる
                    if vehInfo_by_current_vehID.has_tsunami_precursor_info() and not agent_by_current_vehID.get_normalcy_lane_change_motivation_flg():
                        current_lane_change_motivation = agent_by_current_vehID.get_lane_change_threshold_list()[-1]
                        agent_by_current_vehID.append_lane_change_threshold_list(current_lane_change_motivation-agent_by_current_vehID.get_normalcy_lane_change_motivation()) # TODO 仮置き
                        agent_by_current_vehID.append_time_lane_change_list(traci.simulation.getTime()-agent_by_current_vehID.get_created_time())
                        agent_by_current_vehID.set_normalcy_lane_change_motivation_flg(True)
                    
                    # 同調整バイアスの実装箇所 ここを2で割るとTrueにならない
                    if traci.simulation.getTime() % 5 == 0 and not vehInfo_by_current_vehID.get_arrival_flag() and not agent_by_current_vehID.get_evacuation_route_changed_flg():
                        # 周囲が避難行動を取る場合、自身も避難行動を取ろうとするため、閾値が減少
                        if utilities.is_vehIDs_another_lane(target_vehID=current_vehID, vehInfo_list=vehInfo_list):
                            current_lane_change_motivation = agent_by_current_vehID.get_lane_change_threshold_list()[-1]
                            agent_by_current_vehID.append_lane_change_threshold_list(current_lane_change_motivation-majority_bias_score)
                            agent_by_current_vehID.append_time_lane_change_list(traci.simulation.getTime()-agent_by_current_vehID.get_created_time())
                            POSITIVE_MAJORITY_BIAS_COUNT += 1 

                        # 周囲が避難行動を取らない場合、周囲の行動に同調するため、自身の閾値を上昇させる
                        else:
                            current_lane_change_motivation = agent_by_current_vehID.get_lane_change_threshold_list()[-1]
                            agent_by_current_vehID.append_lane_change_threshold_list(current_lane_change_motivation+majority_bias_score)
                            agent_by_current_vehID.append_time_lane_change_list(traci.simulation.getTime()-agent_by_current_vehID.get_created_time())
                            NEGATIVE_MAJORITY_BIAS_COUNT += 1

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
    traci.vehicle.setSpeed(current_vehID, 3.0)
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
                                                ATTR_RATE=early_rate
                                                )

    # ドライバーの行動の初期化
    utilities.init_driver_behavior(vehIDs = vehID_list, lane_change_mode=1)
    for vehID in traci.vehicle.getIDList():
        traci.vehicle.setMaxSpeed(vehID, 9.0)
    run(majority_bias_score=majority_bias_score)
    print("LANE_CHANGED_VEHICLE_NUM:", LANE_CHANGED_VEHICLE_COUNT)
    print("ROUTE_CHANGED_VEHICLE_NUM:", NEW_VEHICLE_COUNT)
    print("POSITIVE_MAJORITY_BIAS_COUNT:", POSITIVE_MAJORITY_BIAS_COUNT)
    print("NEGATIVE_MAJORITY_BIAS_COUNT:", NEGATIVE_MAJORITY_BIAS_COUNT)
    print(f"arrival_time_by_vehID_dict: {arrival_time_by_vehID_dict}")
    if len(arrival_time_list) == TOTAL_VEHNUM:
        print("OK all vehs arrived ")
    else:
        print(f"NG all vehs not arrived {len(arrival_time_list)}")