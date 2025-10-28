from .VehicleInfo import VehicleInfo

class Agent():
    def __init__(self, vehID:str, target_shelter:str, tunning_threshold:int, route_change_threshold:float, lane_change_init_threshold:float):
        self.vehID = vehID #　車両ID
        self.target_shelter = target_shelter #　車両が向かう避難所
        self.near_edgeID_by_target_shelter = "" #　車両が向かう避難所に接続するedgeID
        self.candidate_edge_by_shelterID = {} #　車両が向かう避難所の候補地
        self.congestion_duration = 0 #　渋滞継続時間
        self.tunning_threshold = tunning_threshold # Agentの行動を変更するstress->tunningへの耐久時間
        self.route_change_threshold = route_change_threshold # Agentの行動を変更するtunnig->change渋滞継続時間の耐久時間
        self.route_change_threhold_list = [] #　耐久時間リスト
        self.current_lane_change_motivation = 0 # 現在の車線変更動機付け
        self.x_lane_change = []
        self.y_lane_change = []
        self.lane_change_xy_dict = {} # 車線変更xy座標辞書
        self.lane_change_init_threshold = lane_change_init_threshold #　車線変更初期閾値
        self.lane_change_threshold_list = [] #　車線変更閾値リスト
        self.time_lane_change_list = [] # 車線変更時間リスト
        self.following_threshold = 0 # 避難地を変更した運転者に追従する閾値
        self.normalcy_lane_change_motivation = 0 # 通常時の車線変更動機付け
        self.created_time = 0 # エージェント作成時間
        self.arrival_time = 0 # 避難地到着時間
        self.created_time_flg = False # エージェント作成時間設定フラグ
        self.shelter_flg = False #　駐車フラグ
        self.shelter_changed_flg = False #　避難地変更フラグ
        self.evacuation_route_changed_flg = False # 避難ルート変更フラグ
        self.normalcy_lane_change_motivation_flg = False # 通常時の車線変更動機付けフラグ

    #　渋滞継続時間の更新
    def update_congestion_duration(self):
        self.congestion_duration += 1

    #　渋滞継続時間のリセット
    def reset_congestion_duration(self):
        self.congestion_duration = 0

    # 候補避難地の初期設定
    def init_set_candidate_near_shelter(self, shelter_edge_by_IDs:dict):
        self.set_candidate_shelter(shelter_edge_by_IDs)

    # 避難地の候補地を更新
    def update_candidate_edge_by_shelterID(self, vehInfo:VehicleInfo):
        if not self.get_candidate_edge_by_shelterID() == {}:
            print(f'shelterID: {shelterID}, near_edge: {near_edge}  vehInfo.get_congestion_level_by_shelter(shelterID): {vehInfo.get_congestion_level_by_shelter(shelterID)}')
        # 混雑情報を取得
        new_candidate_edge_by_shelterID = {}
        for shelterID, near_edge in self.get_candidate_edge_by_shelterID().items():
            # 候補地ごとの避難地の混雑情報を取得し、満杯でない避難地のみを残す
            if not vehInfo.get_congestion_level_by_shelter(shelterID) > 0.99:
                new_candidate_edge_by_shelterID[shelterID] = near_edge
        # 候補地が0になってしまう場合は、現在の避難地を残すように変更
        self.set_candidate_shelter(new_candidate_edge_by_shelterID)

    #　車両IDの取得・設定
    def get_vehID(self):
        return self.vehID
    def set_vehID(self, vehID:str):
        self.vehID = vehID

    #　車両が向かう避難所の取得・設定
    def get_target_shelter(self):
        return self.target_shelter
    def set_target_shelter(self, target_shelter:str):
        self.target_shelter = target_shelter

    #　車両が向かう避難所に接続するedgeIDの取得・設定
    def get_near_edgeID_by_target_shelter(self):
        return self.near_edgeID_by_target_shelter
    def set_near_edgeID_by_target_shelter(self, near_edgeID_by_target_shelter:str):
        self.near_edgeID_by_target_shelter = near_edgeID_by_target_shelter

    # 車両が向かう避難所の候補地の取得・設定
    def get_candidate_edge_by_shelterID(self):
        return self.candidate_edge_by_shelterID
    def set_candidate_edge_by_shelterID(self, candidate_edge_by_shelterID:dict):
        self.candidate_edge_by_shelterID = candidate_edge_by_shelterID

    # 車両が向かう避難所の候補の取得・設定
    def get_candidate_shelter(self):
        return self.candidate_shelter
    def set_candidate_shelter(self, candidate_shelter:list):
        self.candidate_shelter = candidate_shelter

    #　渋滞継続時間の取得・設定
    def get_congestion_duration(self):
        return self.congestion_duration
    def set_congestion_duration(self, congestion_duration:str):
        self.congestion_duration = congestion_duration

    # Agentの行動を変更するstress->tunningへの耐久時間の取得・設定
    def get_tunning_threshold(self):
        return self.tunning_threshold
    def set_tunning_threshold(self, tunning_threshold:int):
        self.tunning_threshold = tunning_threshold

    # Agentの行動を変更するtunnig->change渋滞継続時間の耐久時間の取得・設定
    def get_route_change_threshold(self):
        return self.route_change_threshold
    def set_route_change_threshold(self, route_change_threshold:int):
        self.route_change_threshold = route_change_threshold

    # 現在の車線変更動機付けの取得・設定
    def get_current_lane_change_motivation(self):
        return self.current_lane_change_motivation
    def set_current_lane_change_motivation(self, current_lane_change_motivation:int):
        self.current_lane_change_motivation = current_lane_change_motivation

    # 車線変更初期閾値の取得・設定
    def get_lane_change_init_threshold(self):
        return self.lane_change_init_threshold
    def set_lane_change_init_threshold(self, lane_change_init_threshold:int):
        self.lane_change_init_threshold = lane_change_init_threshold
    
    # 車線変更閾値リストの取得・設定
    def init_lane_change_threshold_list(self):
        self.lane_change_threshold_list.append(self.get_lane_change_init_threshold())
    def get_lane_change_threshold_list(self):
        return self.lane_change_threshold_list
    def set_lane_change_threshold_list(self, lane_change_threshold_list:list):
        self.lane_change_threshold_list = lane_change_threshold_list
    def append_lane_change_threshold_list(self, value:float):
        self.lane_change_threshold_list.append(value)
    
    # 車線変更時間リストの取得・設定
    def init_time_lane_change_list(self):
        self.time_lane_change_list.append(0)
    def get_time_lane_change_list(self):
        return self.time_lane_change_list
    def set_time_lane_change_list(self, time_lane_change_list:list):
        self.time_lane_change_list = time_lane_change_list
    def append_time_lane_change_list(self, value:int):
        self.time_lane_change_list.append(value)

    # 車線変更x座標の取得・設定
    def get_x_lane_change(self):
        return self.x_lane_change
    def set_x_lane_change(self, x_lane_change:dict):
        self.x_lane_change = x_lane_change
    def append_x_lane_change(self, value:float):
        self.x_lane_change.append(value)
    def pop_x_lane_change(self):
        return self.x_lane_change.pop()

    # 車線変更y座標の取得・設定
    def get_y_lane_change(self):
        return self.y_lane_change
    def set_y_lane_change(self, y_lane_change:dict):
        self.y_lane_change = y_lane_change
    def append_y_lane_change(self, value:float):
        self.y_lane_change.append(value)
    def pop_y_lane_change(self):
        return self.y_lane_change.pop()
    
    # 車線変更xy座標辞書の取得・設定
    def get_lane_change_xy_dict(self):
        return self.lane_change_xy_dict
    def set_lane_change_xy_dict(self, lane_change_xy_dict:dict):
        self.lane_change_xy_dict = lane_change_xy_dict

    # 避難地を変更した運転者に追従する閾値の取得・設定
    def get_following_threshold(self):
        return self.following_threshold
    def set_following_threshold(self, following_threshold:int):
        self.following_threshold = following_threshold
    
    # 正常性バイアスによるモチベの下がり具合の取得・設定
    def get_normalcy_lane_change_motivation(self):
        return self.normalcy_lane_change_motivation
    def set_normalcy_lane_change_motivation(self, normalcy_lane_change_motivation:int):
        self.normalcy_lane_change_motivation = normalcy_lane_change_motivation
    
    # エージェント作成時間の取得・設定
    def get_created_time(self):
        return self.created_time
    def set_created_time(self, created_time:int):
        self.created_time = created_time
    
    # 避難地到着時間の取得・設定
    def get_arrival_time(self):
        return self.arrival_time
    def set_arrival_time(self, arrival_time:int):
        self.arrival_time = arrival_time
    
    # エージェント作成時間設定フラグの取得・設定
    def get_created_time_flg(self):
        return self.created_time_flg
    def set_created_time_flg(self, created_time_flg:bool):
        self.created_time_flg = created_time_flg

    #　駐車フラグの取得・設定
    def get_shelter_flg(self):
        return self.shelter_flg
    def set_shelter_flg(self, shelter_flg:bool):
        self.shelter_flg = shelter_flg

    #　避難地変更フラグの取得・設定
    def get_shelter_changed_flg(self):
        return self.shelter_changed_flg
    def set_shelter_changed_flg(self, shelter_changed_flg:bool):
        self.shelter_changed_flg = shelter_changed_flg
    
    # 避難ルート変更フラグの取得・設定
    def get_evacuation_route_changed_flg(self):
        return self.evacuation_route_changed_flg
    def set_evacuation_route_changed_flg(self, evacuation_route_changed_flg:bool):
        self.evacuation_route_changed_flg = evacuation_route_changed_flg
    
    # 通常時の車線変更動機付けフラグの取得・設定
    def get_normalcy_lane_change_motivation_flg(self):
        return self.normalcy_lane_change_motivation_flg
    def set_normalcy_lane_change_motivation_flg(self, normalcy_lane_change_motivation_flg:bool):
        self.normalcy_lane_change_motivation_flg = normalcy_lane_change_motivation_flg

    #　エージェントの情報を表示
    def print_info(self):
        print(f'vehID: {self.vehID}, target_shelter: {self.target_shelter}, near_edgeID_by_target_shelter: {self.near_edgeID_by_target_shelter}, candidate_shelter: {self.candidate_shelter}, congestion_duration: {self.congestion_duration}, tunning_threshold: {self.tunning_threshold}, route_change_threshold: {self.route_change_threshold}, following_threshold: {self.following_threshold}, shelter_flg: {self.shelter_flg}, shelter_changed_flg: {self.shelter_changed_flg}')
