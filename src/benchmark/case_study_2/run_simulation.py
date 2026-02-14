"""
Decentralized Variable Speed Limit Control
-------------------------------------------
Authors:        Kevin Riehl, Davide Pusino
Organization:   ETH Zürich, Switzerland
Date:           2024, June 30th
Submitted to:   Advanced Topics in Control (lecture 227-0690-12 FS2024)
-------------------------------------------
Description of software:
    This code runs a Python-controlled simulation of the traffic microsimulation
    software SUMO (using traci interface).
    The programm simulates the generation of traffic, vehicle-2-vehicle 
    communication using dedicated short range communication (DSRC),
    a discrete-time consensus algorithm, a gossip algorithm, and compliant
    vehicles that follow a Bellman-like variable speed limit (VSL) control law.
Usage:
    - Please specify your installation 
"""




# *****************************************************************************
# ******* IMPORTS *************************************************************
# *****************************************************************************
import os
import sys
if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci
import numpy as np
import pandas as pd




# *****************************************************************************
# ******* RESET RANDOM SEED ***************************************************
# *****************************************************************************
np.random.seed(0)




# *****************************************************************************
# ******* SUMO FUNCTIONS ******************************************************
# *****************************************************************************
def startSumo(sumo_binary_path, sumo_config_file):
    """This function starts SUMO simulator"""
    sumo_run_command = [sumo_binary_path, "-c", sumo_config_file, "--start", "--quit-on-end", "--time-to-teleport", "-1"]
    traci.start(sumo_run_command)

def stopSumo():
    """This function stops SUMO simulator"""
    traci.close()

def spawnProcedure(flow_per_route, flow_ramp, vehicle_counter, vehicle_infos):
    """This function spawns vehicles per each of the four routes using exponenetially distributed random numbers."""
    for route in routes:
        if route=="ramp":
            spawn_prob = flow_ramp/60/60
        else:
            spawn_prob = flow_per_route/60/60
        rand_exp_variable = min(1, np.random.exponential(1/5))
        # CDF of exp. variable CDF(x) = 1-e^(-lambda*x)
        if rand_exp_variable <= -np.log(1-spawn_prob)/5:
            new_veh_id = "v"+str(vehicle_counter)
            traci.vehicle.add(new_veh_id, route, typeID=np.random.choice(vehicle_types))
            vehicle_infos[new_veh_id] = {
                "isConnected": np.random.choice([True, False], p=[SHARE_CONNECTED, 1-SHARE_CONNECTED]),
                "isCompliant": False,
                "memory": {
                    "consensus_epoch": 0, 
                    "opinion": -1,
                    "value_time": -1,
                    "in-degree": 0,
                },
                "mailbox": [],
                "actuated": 0,
            }
            if vehicle_infos[new_veh_id]["isConnected"]:
                vehicle_infos[new_veh_id]["isCompliant"] = np.random.choice([True, False], p=[SHARE_COMPLIANT, 1-SHARE_COMPLIANT])
            vehicle_counter += 1
    return vehicle_counter, vehicle_infos

def retrieveVehicleInformationFromSUMO():
    """This function retrieves relevant vehicle information from SUMO"""
    vehicle_data = []
    for vehicle_id in traci.vehicle.getIDList():
        vehicle_data.append([
            vehicle_id,
            traci.vehicle.getVehicleClass(vehicle_id),
            traci.vehicle.getPosition(vehicle_id)[0],
            traci.vehicle.getPosition(vehicle_id)[1],
            traci.vehicle.getSpeed(vehicle_id) * 3.6
        ])
    return vehicle_data

def determineCurrentVehicleRegion(vehicle_data):
    """This function determines the current region of each vehicle."""
    for vehicle_record in vehicle_data:
        if vehicle_record[2] < ROI_X[0]:
            vehicle_record.append(REGION_CLASS_NONE)
        elif vehicle_record[2] > ROI_X[0] and vehicle_record[2] < ROI_X[1]:
            vehicle_record.append(REGION_CLASS_ACTUATE)
        elif vehicle_record[2] > ROI_X[1] and vehicle_record[2] < ROI_X[2]:
            vehicle_record.append(REGION_CLASS_SENSE)
        else:
            vehicle_record.append(REGION_CLASS_NONE2)
    return vehicle_data

def determineCurrentVehicleNeighbours(vehicle_data, vehicle_infos):
    """This function determines the current neighbors of each vehicle."""
    for vehicle_record_a in vehicle_data:
        vehicle_id_a = vehicle_record_a[0]
        if vehicle_infos[vehicle_id_a]["isConnected"] and vehicle_infos[vehicle_id_a]["active"]:
            position_a = [vehicle_record_a[2], vehicle_record_a[3]]
            lst_neighbours_a = []
            for vehicle_record_b in vehicle_data:
                vehicle_id_b = vehicle_record_b[0]
                if vehicle_id_a != vehicle_id_b:
                    if vehicle_infos[vehicle_id_b]["isConnected"] and vehicle_infos[vehicle_id_b]["active"]:
                        position_b = [vehicle_record_b[2], vehicle_record_b[3]]
                        distance = np.linalg.norm(np.asarray(position_a)-np.asarray(position_b))
                        if distance <= DSRC_COMM_DIST:
                            lst_neighbours_a.append(vehicle_id_b)
            vehicle_record_a.append(lst_neighbours_a)
        else:
            vehicle_record_a.append([])
    return vehicle_data

def colorateVehiclesBasedOnRegion(vehicle_data):
    """This function colors vehicles depending on which region they are in."""
    for vehicle_record in vehicle_data:
        vehicle_id = vehicle_record[0]
        vehicle_roi = vehicle_record[5]
        if vehicle_roi == REGION_CLASS_NONE or vehicle_roi == REGION_CLASS_NONE2:
            traci.vehicle.setColor(vehicle_id, (200, 200, 200))
        elif vehicle_roi == REGION_CLASS_ACTUATE:
            traci.vehicle.setColor(vehicle_id, (255, 0, 0))
        elif vehicle_roi == REGION_CLASS_SENSE:
            traci.vehicle.setColor(vehicle_id, (0, 255, 255))

def processVehicleInformation(vehicle_data, vehicle_infos):
    """This function processes the vehicle information from SUMO and adds features such as ROI and neighbours."""
    vehicle_data = determineCurrentVehicleRegion(vehicle_data)
    vehicle_data = determineCurrentVehicleNeighbours(vehicle_data, vehicle_infos)
    colorateVehiclesBasedOnRegion(vehicle_data)
    return vehicle_data

def updateCurrentVehicleInformation(vehicle_infos, vehicle_data):
    """This function updates the vehicle information stored in current vehicle information variable based on the vehicle information data from SUMO."""
    for vehicle_id in vehicle_infos:
        found_id = False
        for vehicle_record in vehicle_data:
            if vehicle_record[0] == vehicle_id:
                found_id = True       
        vehicle_infos[vehicle_id]["active"] = found_id
    for vehicle_record in vehicle_data:
        vehicle_id = vehicle_record[0]
        if "region" in vehicle_infos[vehicle_id] and vehicle_infos[vehicle_id]["region"] == REGION_CLASS_ACTUATE and vehicle_record[5] == REGION_CLASS_SENSE:
                vehicle_infos[vehicle_id]["memory"]["opinion"] = vehicle_record[4]
        vehicle_infos[vehicle_id]["region"] = vehicle_record[5]
        vehicle_infos[vehicle_id]["Speed"] = vehicle_record[4]
        vehicle_infos[vehicle_id]["Pos"] = [vehicle_record[2], vehicle_record[3]]
        vehicle_infos[vehicle_id]["VType"] = vehicle_record[1]
        vehicle_infos[vehicle_id]["neighbours"] = vehicle_record[6]
    return vehicle_infos

def getListOfActiveVehicleIDs(vehicle_infos):
    """This function determines a list of active vehicles, meaning vehicles that did not disappear from simulation yet."""
    lst_veh_ids = []
    for vehicle_id in vehicle_infos:
        if vehicle_infos[vehicle_id]["active"]:
            lst_veh_ids.append(vehicle_id)
    return lst_veh_ids

def getListOfActiveVehicleIDsInROI(vehicle_infos):
    """This function determines a list of active vehicles that are in region of interest (ROI)."""
    lst_veh_ids = []
    for vehicle_id in vehicle_infos:
        if vehicle_infos[vehicle_id]["active"] and vehicle_infos[vehicle_id]["region"] == REGION_CLASS_SENSE:
            lst_veh_ids.append(vehicle_id)
    return lst_veh_ids

def getListOfActiveVehicleIDsBeforeROI(vehicle_infos):
    """This function determines a list of active vehicles that are in region of interest (ROI)."""
    lst_veh_ids = []
    for vehicle_id in vehicle_infos:
        if vehicle_infos[vehicle_id]["active"] and vehicle_infos[vehicle_id]["region"] == REGION_CLASS_ACTUATE:
            lst_veh_ids.append(vehicle_id)
    return lst_veh_ids

def getListOfActiveConnectedVehicles(vehicle_infos):
    """This function determines a list of active vehicles that are connected (have ability to communicate)."""
    lst_active_veh_ids = []
    for vehicle_id in vehicle_infos:
        if vehicle_infos[vehicle_id]["active"] and vehicle_infos[vehicle_id]["isConnected"]:
            lst_active_veh_ids.append(vehicle_id)
    return lst_active_veh_ids

def getListOfActiveConnectedVehiclesInROIAndBefore(vehicle_infos):
    """This function determines a list of active vehicles that are connected (have ability to communicate)."""
    lst_active_veh_ids = []
    for vehicle_id in vehicle_infos:
        if vehicle_infos[vehicle_id]["active"] and vehicle_infos[vehicle_id]["isConnected"]:
            if vehicle_infos[vehicle_id]["region"] == REGION_CLASS_ACTUATE or vehicle_infos[vehicle_id]["region"] == REGION_CLASS_SENSE:
                lst_active_veh_ids.append(vehicle_id)
    return lst_active_veh_ids

def recordTrafficStateInformation(historic_traffic_info, vehicle_infos, memory_roi_vehicles):
    """This function records historic traffic information related to vehicles that are in the ROI."""
    # Get Simulation Time
    time = traci.simulation.getTime()
    # Retrieve vehicle information in ROI
    lst_active_veh_ids_roi = getListOfActiveVehicleIDsInROI(vehicle_infos)
    lst_active_veh_ids_bef = getListOfActiveVehicleIDsBeforeROI(vehicle_infos)
    # Speed Calculation
    vehicle_speeds = []
    for vehicle_id in lst_active_veh_ids_roi:
        vehicle_speeds.append(vehicle_infos[vehicle_id]["Speed"])
    av_speed = np.nanmean(vehicle_speeds) # [km/h]
    vehicle_speeds_bef = []
    for vehicle_id in lst_active_veh_ids_bef:
        vehicle_speeds_bef.append(vehicle_infos[vehicle_id]["Speed"])
    av_speed_bef = np.nanmean(vehicle_speeds_bef) # [km/h]
    num_vehicle_bef = len(lst_active_veh_ids_bef)
    # Density Calculation
    roi_length = (ROI_X[2]-ROI_X[1])/4/1000  # there are four lanes
    density = len(vehicle_speeds)/(roi_length)  # [veh/km]
    # Flow / Vehicles Left from ROI Calculation
    vehicles_left = 0
    vehicles_to_delete = []
    for veh_id in memory_roi_vehicles:
        if veh_id not in lst_active_veh_ids_roi:
            vehicles_left += 1
            vehicles_to_delete.append(veh_id)
    for veh_id in vehicles_to_delete:
        memory_roi_vehicles.remove(veh_id)
    for veh_id in lst_active_veh_ids_roi:
        if veh_id not in memory_roi_vehicles:
            memory_roi_vehicles.append(veh_id)
    # Append new record to dataframe
    new_line = pd.DataFrame({"speed": av_speed, "density": density,
                             "vehicles_left": vehicles_left, "speed_bef": av_speed_bef, "n_veh_bef": num_vehicle_bef}, index=[time])
    historic_traffic_info = pd.concat([historic_traffic_info, new_line], axis=0)
    return historic_traffic_info, memory_roi_vehicles

def getTrafficStateHistory(traffic_state_history_df, averaging_period):
    """This function calculates the historic ground truth traffic state based on the historic traffic information, using a time averaging approach."""
    traffic_state_history_df["speed_avg"] = traffic_state_history_df["speed"].rolling(
        window=averaging_period, min_periods=1).mean()
    traffic_state_history_df["speed_bef_avg"] = traffic_state_history_df["speed_bef"].rolling(
        window=averaging_period, min_periods=1).mean()
    traffic_state_history_df["n_veh_bef_avg"] = traffic_state_history_df["n_veh_bef"].rolling(
        window=averaging_period, min_periods=1).mean()
    traffic_state_history_df["density_avg"] = traffic_state_history_df["density"].rolling(
        window=averaging_period, min_periods=1).mean()
    traffic_state_history_df["flow_avg"] = traffic_state_history_df["vehicles_left"].rolling(
        window=averaging_period, min_periods=1).sum()
    traffic_state_history_df["flow_avg"] = traffic_state_history_df["flow_avg"]/averaging_period*(60*60)
    return traffic_state_history_df




# *****************************************************************************
# ******* COMMUNICATION FUNCTIONS *********************************************
# *****************************************************************************
def executeCommunicationRound(vehicle_infos):
    """This function executes one communication round across all vehicles."""
    # Determine Communication Topology
    lst_communicating_veh_ids, matrix_binary, num_com_vehicles, num_com_vehicles_roi, num_com_vehicles_before, num_connections = determineCommunicationStatistics(vehicle_infos)
    # Prepare What each Agent sends out to its neighbours
    vehicle_infos = prepareMessages(lst_communicating_veh_ids, vehicle_infos)
    # Send Messages to each Neighboring Agent
    vehicle_infos, num_msgs_sent_roi, num_msgs_sent_not_roi = transceiveMessages(lst_communicating_veh_ids, vehicle_infos)
    # Process Received Messages
    vehicle_infos = processReceivedMessages(lst_communicating_veh_ids, vehicle_infos)
    # Clean Mailboxes for next round
    vehicle_infos = cleanMailboxes(lst_communicating_veh_ids, vehicle_infos)
    # Summarize connection statistics
    comm_stats = [num_com_vehicles, num_com_vehicles_roi, num_com_vehicles_before, num_connections, num_msgs_sent_roi, num_msgs_sent_not_roi]
    return vehicle_infos, comm_stats

def determineCommunicationStatistics(vehicle_infos):
    lst_communicating_veh_ids = getListOfActiveConnectedVehiclesInROIAndBefore(vehicle_infos)
    lst_connected_veh_roi = [vehicle_infos[vehicle_id]["region"] == REGION_CLASS_SENSE for vehicle_id in lst_communicating_veh_ids]
    # Determine binary adjacency matrix
    matrix_binary = np.zeros((len(lst_communicating_veh_ids), len(lst_communicating_veh_ids)))
    countA = 0
    for vehicle_a in lst_communicating_veh_ids:
        countB = 0
        for vehicle_b in lst_communicating_veh_ids:
            if vehicle_b in vehicle_infos[vehicle_a]["neighbours"]:
                matrix_binary[countA][countB] = 1
            countB += 1
        countA += 1
    # Determine number of connected vehicles
    num_com_vehicles = len(lst_communicating_veh_ids)
    num_com_vehicles_roi = np.sum(lst_connected_veh_roi)
    num_com_vehicles_before = num_com_vehicles - num_com_vehicles_roi
    # Determine connections
    num_connections = int(np.sum(matrix_binary)/2)    
    return lst_communicating_veh_ids, matrix_binary, num_com_vehicles, num_com_vehicles_roi, num_com_vehicles_before, num_connections

def prepareMessages(lst_veh_ids, vehicle_infos):
    """This function prepares for each vehicle the message it will send out to others."""
    time = traci.simulation.getTime()
    for vehicle_id in lst_veh_ids:
        message_to_send_out = {
            "time": time,
            "region": vehicle_infos[vehicle_id]["region"],
            "value" : -1, # if value not set (-1), then vehicle does not send out message
            "value_time": -1,
            "in-degree": vehicle_infos[vehicle_id]["memory"]["in-degree"]
        }
        value = vehicle_infos[vehicle_id]["memory"]["opinion"]
        message_to_send_out["value"] = value
        if vehicle_infos[vehicle_id]["region"] == REGION_CLASS_ACTUATE:
            value_time = vehicle_infos[vehicle_id]["memory"]["value_time"]   
            message_to_send_out["value_time"] = value_time
        vehicle_infos[vehicle_id]["message"] = message_to_send_out
    return vehicle_infos

def transceiveMessages(lst_veh_ids, vehicle_infos):
    """This function transceives messages, meaning it exchanges the messages and puts them into the mailbox of vehicles."""
    num_msgs_sent_roi = 0
    num_msgs_sent_not_roi = 0
    for vehicle_id_sender in lst_veh_ids:
        for vehicle_id_receiver in vehicle_infos[vehicle_id_sender]["neighbours"]:
            if vehicle_infos[vehicle_id_receiver]["region"] == REGION_CLASS_SENSE or vehicle_infos[vehicle_id_receiver]["region"] == REGION_CLASS_ACTUATE:
                if vehicle_infos[vehicle_id_sender]["message"]["value"] != -1: # only if it was populated send out
                    vehicle_infos[vehicle_id_receiver]["mailbox"].append(vehicle_infos[vehicle_id_sender]["message"])
                    if vehicle_infos[vehicle_id_sender]["region"] == REGION_CLASS_SENSE:
                        num_msgs_sent_roi += 1
                    elif vehicle_infos[vehicle_id_sender]["region"] == REGION_CLASS_ACTUATE:
                        num_msgs_sent_not_roi += 1
    return vehicle_infos, num_msgs_sent_roi, num_msgs_sent_not_roi

def processReceivedMessages(lst_veh_ids, vehicle_infos):
    """This function processes for each vehicle the messages it received."""
    time = traci.simulation.getTime()
    for vehicle_id in lst_veh_ids:
        if vehicle_infos[vehicle_id]["region"] == REGION_CLASS_SENSE:
            vehicle_infos[vehicle_id]["memory"]["opinion"] = processReceivedMessageVehicleInROI(vehicle_infos[vehicle_id], vehicle_infos[vehicle_id]["mailbox"], time)
            vehicle_infos[vehicle_id]["memory"]["in-degree"] = len(vehicle_infos[vehicle_id]["mailbox"])
        elif vehicle_infos[vehicle_id]["region"] == REGION_CLASS_ACTUATE:
            new_opinion, new_value_time = processReceivedMessageVehicleBeforeROI(vehicle_infos[vehicle_id], vehicle_infos[vehicle_id]["mailbox"], time)    
            vehicle_infos[vehicle_id]["memory"]["opinion"] = new_opinion
            vehicle_infos[vehicle_id]["memory"]["value_time"] = new_value_time
            vehicle_infos[vehicle_id]["memory"]["in-degree"] = 0
    return vehicle_infos

def processReceivedMessageVehicleInROI(vehicle_info_record, vehicle_mailbox, time):
    """This function updates the opinion for a specific vehicle in ROI based on its memory and received messages."""
    new_opinion = -1
    values = []
    weights = []
    # Add own speed
    values.append(vehicle_info_record["Speed"])
    weights.append(vehicle_info_record["memory"]["in-degree"]+1)
    # Add received opinions
    for msg in vehicle_mailbox:
        # Consider messages only from ROI
        if msg["region"] == REGION_CLASS_SENSE:
            values.append(msg["value"])
            weights.append(msg["in-degree"]+1)
    new_opinion = np.average(values, weights=weights)
    return new_opinion
    
def processReceivedMessageVehicleBeforeROI(vehicle_info_record, vehicle_mailbox, time):
    """This function updates the opinion for a specific vehicle before ROI based on its memory and received messages."""
    old_opinion = vehicle_info_record["memory"]["opinion"]
    if vehicle_info_record["memory"]["value_time"] == -1:
        old_opinion = -1
    elif time - vehicle_info_record["memory"]["value_time"] <= MAX_CONSIDERED_INFO_AGE:
        old_opinion = -1
    # Determine if messages where received, if not, stick to old opinion
    if len(vehicle_mailbox)==0:
        return old_opinion, time
    # Determine values received by ROI vehicles, and consider there average as truth
    roi_values = []
    for msg in vehicle_mailbox:
        if msg["region"] == REGION_CLASS_SENSE:
            roi_values.append(msg["value"])
    if len(roi_values)>0:
        return np.nanmean(roi_values), time
    # Otherwise listen to other not-ROI vehicles, and take most recent information available from neighbourhood
    not_roi_values = []
    not_roi_times = []
    for msg in vehicle_mailbox:
        if msg["region"] == REGION_CLASS_ACTUATE and msg["value"] != -1 and msg["value_time"] != -1:
            if time - msg["value_time"] <= MAX_CONSIDERED_INFO_AGE:
                not_roi_values.append(msg["value"])
                not_roi_times.append(msg["value_time"])
    if len(not_roi_times)==0:
        return old_opinion, time
    most_recent_time = np.nanmax(not_roi_times)
    most_recent_values = np.asarray(not_roi_values)[np.asarray(not_roi_times)==most_recent_time]
    new_opinion = np.nanmean(most_recent_values)
    new_time_value = most_recent_time
    return new_opinion, new_time_value

def cleanMailboxes(lst_veh_ids, vehicle_infos):
    """This function empties the mailboxes, so that the mailboxes are empty at the beginning of an communication round."""
    # Clean Mailboxes
    for vehicle_id in lst_veh_ids:
        vehicle_infos[vehicle_id]["mailbox"] = []
    return vehicle_infos

def recordOpinions(history_opinion_infos, current_vehicle_infos, time):
    opinions_roi = {}
    opinions_before = {}
    opinions_positions = {}
    opinions_value_time = {}
    num_actuated = 0
    for vehicle_id in current_vehicle_infos:
        num_actuated += current_vehicle_infos[vehicle_id]["actuated"]
        if "region" in current_vehicle_infos[vehicle_id] and "Pos" in current_vehicle_infos[vehicle_id]:
            if current_vehicle_infos[vehicle_id]["region"] == REGION_CLASS_SENSE:
                opinions_roi[vehicle_id] = current_vehicle_infos[vehicle_id]["memory"]["opinion"]
            elif current_vehicle_infos[vehicle_id]["region"] == REGION_CLASS_ACTUATE:
                opinions_before[vehicle_id] = current_vehicle_infos[vehicle_id]["memory"]["opinion"]
                opinions_value_time[vehicle_id] = current_vehicle_infos[vehicle_id]["memory"]["value_time"]
            opinions_positions[vehicle_id] = current_vehicle_infos[vehicle_id]["Pos"]
    history_opinion_infos[time] = {}
    history_opinion_infos[time]["before"] = opinions_before
    history_opinion_infos[time]["roi"] = opinions_roi
    history_opinion_infos[time]["positions"] = opinions_positions
    history_opinion_infos[time]["value_times"] = opinions_value_time
    history_opinion_infos[time]["actuated"] = num_actuated
    return history_opinion_infos




# *****************************************************************************
# ******* CONTROL FUNCTIONS *********************************************
# *****************************************************************************
def executeControl(vehicle_infos):
    """This function applies a control algorithm to all active vehicles that are compliant."""
    lst_active_vehicles = getListOfActiveVehicleIDs(vehicle_infos)
    for vehicle_id in lst_active_vehicles:
        if vehicle_infos[vehicle_id]["isCompliant"]:
            actuated = controlVehicle(vehicle_id, vehicle_infos[vehicle_id])
            vehicle_infos[vehicle_id]["actuated"] = actuated
    return vehicle_infos

def controlVehicle(vehicle_id, vehicle_info_record):
    """This function controls a specific vehicle."""
    actuated_next = 0
    if "region" in vehicle_info_record:
        vehicle_type = vehicle_info_record["VType"]
        vehicle_max_speed = vehicle_type_default_max_speed[vehicle_type] / 3.6
        vehicle_ctr_speed = vehicle_type_default_max_speed[vehicle_type] * SPEED_CONTROL_SLOW_FACTOR / 3.6
        # VEHICLES IN ACTUATION AREA
        if vehicle_info_record["region"] == REGION_CLASS_ACTUATE:
            if vehicle_info_record["memory"]["opinion"] != -1:
                if vehicle_info_record["memory"]["opinion"] < SPEED_CONTROL_THRESHOLD:
                    if vehicle_info_record["actuated"] == 0:
                        traci.vehicle.setMaxSpeed(vehicle_id, vehicle_ctr_speed)
                        actuated_next = 1
                else:
                    if vehicle_info_record["actuated"] == 1:
                        traci.vehicle.setMaxSpeed(vehicle_id, vehicle_max_speed)
            else:
                if vehicle_info_record["actuated"] == 1:
                    traci.vehicle.setMaxSpeed(vehicle_id, vehicle_max_speed)
        # VEHICLES IN SENSOR AREA
        elif vehicle_info_record["region"] == REGION_CLASS_SENSE:
            if vehicle_info_record["actuated"] == 1:
                traci.vehicle.setMaxSpeed(vehicle_id, vehicle_max_speed)
        # VEHICLES IN OTHER AREAS
        else: # ACCELERATE back again
            if vehicle_info_record["actuated"] == 1:
                traci.vehicle.setMaxSpeed(vehicle_id, vehicle_max_speed)
    return actuated_next




# *****************************************************************************
# ******* MAP / SUMO SPECIFIC CONSTANTS ***************************************
# *****************************************************************************
    # SUMO & MAP SPECIFIC
routes = [  # every spawned vehicle will get a random route from this list
    "A_A", "A_B", "A_C", "A_D",
    "B_A", "B_B", "B_C", "B_D",
    "C_A", "C_B", "C_C", "C_D",
    "D_A", "D_B", "D_C", "D_D",
    "ramp"
]
vehicle_types = [  # every spawned vehicle will get a random vehicle type from this list
    "car_A", "car_B", "car_C", "car_D", "car_E",
    "car_A2", "car_B2", "car_C2", "car_D2", "car_E2",
    "del_A", "del_B",
    "del_A2", "del_B2",
    "bus_A", "bus_B",
    "trk_A", "trk_B"
]
vehicle_type_default_max_speed = { # [km/h]
    "passenger": 200,
    "delivery": 200, 
    "bus": 85,
    "truck": 130,
}
    # MAP GEOMETRY SPECIFIC
ROI_X = [-850, 700, 1260]  # Class -1 nothing, Class 0 - control area [<700], Class 1 - consensus area [>700&<1250], Class 2 - nothing [>1250]
REGION_CLASS_NONE  = -1
REGION_CLASS_NONE2 = 2
REGION_CLASS_ACTUATE = 0
REGION_CLASS_SENSE = 1
    # DATA PROCESSING & ANALYSIS
TRAFFIC_STATE_AVERAGE_TIME = 10 # [sec]



# *****************************************************************************
# ******* PARAMETERS **********************************************************
# *****************************************************************************
    # VEHICLE SPAWNING SPECIFIC
SHARE_CONNECTED = 0.5  # [%] how many of vehicles are connected via DSRC
SHARE_COMPLIANT = 1.0  # [%] how many of the connected vehicles comply with control
# SPAWN_FLOW_PER_ROUTE = 180 # [veh/h] is one fourth of per lane in-flow reported in paper
    # COMMUNICATION
DSRC_COMM_DIST = 200  # [m] maximum communication distance for DSRC
DSRC_COMM_PERIOD = 2  # [sec] every 2 s transaction of message takes place
DSRC_COMM_ITERATIONS_PER_ROUND = 1 # if DSRC_COMM_PERIOD = 1, and want more than once per second, then increase here
MAX_CONSIDERED_INFO_AGE = 2000 # [sec] maximum considered information age, if older, not considered anymore by vehicles in acutation section
    # CONTROL
SPEED_CONTROL_THRESHOLD = 80 # [km/h] if estimated ROI speed drops below this threshold, compliant vehicle in actuation section will start to decelerate
SPEED_CONTROL_SLOW_FACTOR = 0.9 # [%] reduce speed to share of vehicle's default max speed (~100km/h)
    # SIMULATION
WARMUP_TIME = 1000      # [sec] simulation run-time to get traffic in equilibrium-state, before recording starts
SIMULATION_TIME = 6000 # [sec] simulation run-time / duration 
SHALL_RECORD_COMMUNI = True # turn on/off recording of communication
SHALL_RECORD_OPINION = True # turn on/off recording of opinion
SHALL_EXECUTE_CONTROL = False # turn on/off control law for compliant vehicles
SHALL_EXECUTE_COMMUNI = True # turn on/off communication between vehicles

def DEMAND_PROFILE(t):
    if it_time < 4000:
        return 160/4*2
    elif it_time < 5000:
        return 200/4*2
    else:
        return 150/4*2

def DEMAND_PROFILE_RAMP(t):
    if it_time < 4000:
        return 160*0.7
    elif it_time < 5000:
        return 200*0.7
    else:
        return 150*0.7


# *****************************************************************************
# ******* SIMULATION RUN ******************************************************
# *****************************************************************************
# Setup Vehicle Information
current_vehicle_infos = {}
history_traffic_infos = pd.DataFrame(columns=["speed", "density", "vehicles_left", "speed_bef"])
history_communi_infos = []
history_opinion_infos = {}
temp_buffer_roi_vehicle_ids = []

# Start Sumo
#sumo_binary_path = "C:/Program Files (x86)/Eclipse/Sumo/bin/sumo.exe" # sumo-gui.exe
sumo_binary_path = "C:/Users/kriehl/AppData/Local/sumo-1.19.0/bin/sumo.exe"
sumo_config_file = "Configuration.sumocfg" 
startSumo(sumo_binary_path, sumo_config_file)
print("Started SUMO")

# Warmup Run
vehicle_counter = 0
for it_time in range(0, SIMULATION_TIME):
    if it_time%10==0:
        print(it_time, SIMULATION_TIME)
    # time.sleep(0.01)
    # Spawn Vehicles
    vehicle_counter, current_vehicle_infos = spawnProcedure(DEMAND_PROFILE(it_time), DEMAND_PROFILE_RAMP(it_time), vehicle_counter, current_vehicle_infos)
    # Retrieve Information about vehicles
    iteration_vehicle_data = retrieveVehicleInformationFromSUMO()
    iteration_vehicle_data = processVehicleInformation(iteration_vehicle_data, current_vehicle_infos)
    current_vehicle_infos = updateCurrentVehicleInformation(current_vehicle_infos, iteration_vehicle_data)
    # Retrieve Information about traffic state
    history_traffic_infos, temp_buffer_roi_vehicle_ids = recordTrafficStateInformation(history_traffic_infos, current_vehicle_infos, temp_buffer_roi_vehicle_ids)
    # Do Communication & Control regularly
    if it_time % DSRC_COMM_PERIOD == 0:
        for num_comm_its in range(0, DSRC_COMM_ITERATIONS_PER_ROUND):
            if SHALL_EXECUTE_COMMUNI:
                current_vehicle_infos, comm_stats = executeCommunicationRound(current_vehicle_infos)
                if SHALL_RECORD_COMMUNI:
                    history_communi_infos.append([it_time, *comm_stats])
                if SHALL_EXECUTE_CONTROL:
                    current_vehicle_infos = executeControl(current_vehicle_infos)
    # Simulate one Time Step
    traci.simulationStep()
    # Record Opinions
    if SHALL_RECORD_OPINION:
        history_opinion_infos = recordOpinions(history_opinion_infos, current_vehicle_infos, it_time)
        
# # Stop Sumo
stopSumo()




# *****************************************************************************
# ******* PROCESS SIMULATION DATA *********************************************
# *****************************************************************************

# Preparing Traffic Data
history_communi_infos = pd.DataFrame(history_communi_infos, columns=["time", "num_com_vehicles", "num_com_vehicles_roi", "num_com_vehicles_before", "num_connections", "num_msgs_sent_roi", "num_msgs_sent_not_roi"])
history_traffic_state = getTrafficStateHistory(history_traffic_infos, TRAFFIC_STATE_AVERAGE_TIME)
avg_roi_speed   = np.nanmean(np.asarray(history_traffic_state["speed_avg"])[WARMUP_TIME:])
avg_roi_flow    = np.nanmean(np.asarray(history_traffic_state["flow_avg"])[WARMUP_TIME:])
avg_roi_density = np.nanmean(np.asarray(history_traffic_state["density_avg"])[WARMUP_TIME:])

# Preparing Communication Statistics
lst_time = list(history_opinion_infos.keys())
lst_share_opinion_roi = []
lst_share_opinion_bef = []
for record_time in history_opinion_infos:
    record = history_opinion_infos[record_time]
    num_bef_vehicles = len(record["before"])
    num_roi_vehicles = len(record["roi"])
    num_have_opinion_bef = 0
    num_have_opinion_roi = 0
    for vehicle_id in record["before"]:
        if record["before"][vehicle_id] != -1:
            num_have_opinion_bef += 1
    for vehicle_id in record["roi"]:
        if record["roi"][vehicle_id] != -1:
            num_have_opinion_roi += 1
    if num_roi_vehicles != 0:
        lst_share_opinion_roi.append(100*num_have_opinion_roi/num_roi_vehicles)
    else:
        lst_share_opinion_roi.append(0)
    if num_bef_vehicles != 0:
        lst_share_opinion_bef.append(100*num_have_opinion_bef/num_bef_vehicles)
    else:
        lst_share_opinion_bef.append(0)
average_have_opinion_roi = np.nanmean(lst_share_opinion_roi[WARMUP_TIME:])
average_have_opinion_bef = np.nanmean(lst_share_opinion_bef[WARMUP_TIME:])

lst_time = list(history_opinion_infos.keys())
lst_true_speed = history_traffic_state["speed_avg"]
lst_mean_absolute_error_roi = []
lst_mean_absolute_error_bef = []
for record_time in history_opinion_infos:
    record = history_opinion_infos[record_time]
    true_speed = lst_true_speed[record_time]
    lst_errors_roi = []
    lst_errors_bef = []
    for vehicle_id in record["before"]:
        if record["before"][vehicle_id] != -1:
            lst_errors_bef.append(abs(record["before"][vehicle_id] - true_speed))
    for vehicle_id in record["roi"]:
        if record["roi"][vehicle_id] != -1:
            lst_errors_roi.append(abs(record["roi"][vehicle_id] - true_speed))
    lst_mean_absolute_error_roi.append(np.nanmean(lst_errors_roi))
    lst_mean_absolute_error_bef.append(np.nanmean(lst_errors_bef))
average_error_roi = np.nanmean(lst_mean_absolute_error_roi[WARMUP_TIME:])
average_error_bef = np.nanmean(lst_mean_absolute_error_bef[WARMUP_TIME:])

lst_time = list(history_opinion_infos.keys())
lst_true_speed = history_traffic_state["speed_avg"]
lst_rel_mean_absolute_error_roi = []
lst_rel_mean_absolute_error_bef = []
lst_num_actuated = []
lst_share_actuated_bef = []
for record_time in history_opinion_infos:
    record = history_opinion_infos[record_time]
    lst_num_actuated.append(history_opinion_infos[record_time]["actuated"])
    lst_share_actuated_bef.append(100 * history_opinion_infos[record_time]["actuated"] / history_traffic_state["n_veh_bef_avg"][record_time])
    true_speed = lst_true_speed[record_time]
    lst_errors_roi = []
    lst_errors_bef = []
    for vehicle_id in record["before"]:
        if record["before"][vehicle_id] != -1:
            lst_errors_bef.append(abs(record["before"][vehicle_id] - true_speed)/true_speed*100)
    for vehicle_id in record["roi"]:
        if record["roi"][vehicle_id] != -1:
            lst_errors_roi.append(abs(record["roi"][vehicle_id] - true_speed)/true_speed*100)
    lst_rel_mean_absolute_error_roi.append(np.nanmean(lst_errors_roi))
    lst_rel_mean_absolute_error_bef.append(np.nanmean(lst_errors_bef))
average_error_rel_roi = np.nanmean(lst_rel_mean_absolute_error_roi[WARMUP_TIME:])
average_error_rel_bef = np.nanmean(lst_rel_mean_absolute_error_bef[WARMUP_TIME:])

lst_time = list(history_opinion_infos.keys())[WARMUP_TIME:]
lst_average_age_info_bef = []
lst_median_age_info_bef = []
lst_max_age_info_bef = []
for record_time in lst_time:
    record = history_opinion_infos[record_time]
    true_time = record_time
    lst_age = []
    for vehicle_id in record["before"]:
        if record["before"][vehicle_id] != -1:
            age = true_time - record["value_times"][vehicle_id]
            lst_age.append(age)
    lst_average_age_info_bef.append(np.nanmean(lst_age))
    lst_median_age_info_bef.append(np.nanmedian(lst_age))
    if len(lst_age)>0:
        lst_max_age_info_bef.append(np.max(lst_age))
    else:
        lst_max_age_info_bef.append(-1)
average_avg_age_bef = np.nanmean(np.asarray(lst_average_age_info_bef))
average_max_age_bef = np.nanmean(np.asarray(lst_max_age_info_bef))




# *****************************************************************************
# ******* PRINT SUMMARY *******************************************************
# *****************************************************************************
print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
print("++++ Traffic State on Average ++++")
print("Vehicles", "\t", len(current_vehicle_infos))
print("Speed", "\t", avg_roi_speed, "km/h")
print("Flow", "\t", avg_roi_flow, "veh/h")
print("Density", "\t", avg_roi_density, "veh/km")
print("TTT", "\t", 1.5/avg_roi_speed * len(current_vehicle_infos), "veh/km")
print()
print("++++ Communication Statistics ++++")
print("Have an Opinion in SENSE Area", "\t", average_have_opinion_roi)
print("Have an Opinion in ACTUA Area", "\t", average_have_opinion_bef)
print("Average Age in ACTUA Area", "\t", average_avg_age_bef)
print("Maximum Age in ACTUA Area", "\t", average_max_age_bef)
print()
print("++++ SENSE Area Speed Estimation Mean Absolute Error ++++")
print("Absolute, in SENSE AREA", "\t", average_error_roi)
print("Absolute, in ACTUA AREA", "\t", average_error_bef)
print("Relative, in SENSE AREA", "\t", average_error_rel_roi)
print("Relative, in ACTUA AREA", "\t", average_error_rel_bef)
