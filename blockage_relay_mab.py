'''
This example demonstrates the implementation of blockages in the simulator and the use of relays to avoid  them.
A valid map file that includes blockages must be used to run this simulation.
'''

import wx
import math
import argparse
from argparse import Namespace, ArgumentParser
from typing import Dict, List, Tuple

import operator
import random

from sim.simulation import World
from sim.loc import ScreenXY as XY
from sim.scenario import BaseScenario
from sim.event import Event
from sim.drawable import Drawable
from node.node import BaseNode
from node.mobility import Stationary, StaticPath
from comm.transceiver import UserTransceiver
from comm.transceiver import Transceiver
from comm.channel import DiscModel
from map.mapinfo import MapInfo
import node.type as NodeType
import map.pin as Pin 
from sim.direction import Dir2D
from node.obstacle import Obstacle

import shapely
from shapely.geometry import Polygon, LineString

import matplotlib.pyplot as plt

connection_time_per_second = 0.0
connection_time_list = []
arm_connection_list = []
sector_list = []
counter = 0.0

num_vehicles = 30   # hard coded
sim_duration = 3600 # hard coded in seconds

active_beams = 0    # counts the number of active beams for the base station


####################################################################
## My Transceiver
## - Perform own signal propagation and detection logic
## - Aware of an obstacle
####################################################################

class BSTransceiver(UserTransceiver):
    
    ## class constants or default values
    noise = -90 # dBm
    snr_threshold = 5 # dB

    def __init__(self, node, freq, channel, beam_width, azimuth):
        super().__init__(node,freq)
        self.obstacle_list = None
        self._beam_width = beam_width
        self._azimuth = azimuth
        
        ## condition the angles to within 0 & 360
        while self._azimuth<0: self._azimuth+=360
        while self._azimuth>=360: self._azimuth-=360
        while self._beam_width<0: self._beam_width+=360
        while self._beam_width>=360: self._beam_width-=360

        self._property_list["type"] = "directional"
        self._property_list["radius"] = channel.get_range()
        self._property_list["beam width"] = self._beam_width
        self._property_list["azimuth"] = self._azimuth
        
    def _is_within_sector(self, departure_angle):
        left_edge = self._azimuth - 0.5*self._beam_width
        right_edge = self._azimuth + 0.5*self._beam_width
        return ((departure_angle>=left_edge and departure_angle<=right_edge) or 
                (departure_angle+360>=left_edge and departure_angle+360<=right_edge) or  # wrap-around
                (departure_angle-360>=left_edge and departure_angle-360<=right_edge))    # wrap-around

    def create_signal(self):
        signal = super().create_signal()
        signal.tx_power = 30 # dBm
        return signal

    def set_obstacle(self, obstacle_list):
        self.obstacle_list = obstacle_list

    def is_crossed_obstacle(self, tx_loc, rx_loc) -> bool:
        if self.obstacle_list is None: return False
        point1 = tx_loc.get_xy()
        point2 = rx_loc.get_xy()
        line = [point1, point2]
        shapely_line = shapely.geometry.LineString(line)
        for polygon in self.obstacle_list:
            if(shapely_line.intersects(polygon)):
                return True
        

    def can_detect(self, the_signal) -> bool:
        '''This is the function to perform signal propagation and
        detection logic.'''

        ## retrieve all key info first
        tx_power = the_signal.tx_power      # get the tx power in dBm
        distance = the_signal.distance      # get the distance travelled
        travelling_dir = the_signal.LOS_dir # get the travelling direction
        tx_loc = the_signal.source._node.get("location") # sender location
        rx_loc = self._node.get("location")              # receiver location
        
        ## check if signal is crossing obstacle
        is_crossed = self.is_crossed_obstacle(tx_loc,rx_loc)

        angle = the_signal.LOS_dir + 180 # need opposite direction at the receiver side
        if angle>=360: angle-=360    # wrap-around
        if not self._is_within_sector(angle): return False

        ## apply simple passloss model c0*d^(-α), where
        #  - d is the distance, in our system, 2 pixels = 1m, minimum 1m
        #  - α is the pathloss exponent, 2 for normal, 2.92 when crossing an obstacle
        #  - c0 is the gain/loss
        ## pathloss PL_db = log10(c0) + 10α*log10(d)
        distance /= 2             # convert pixel-to-meter
        if distance<1: distance=1 # min 1m
        alpha = 2 if not is_crossed else 2.92   # set pathloss exponent
        log_c0 = 61.4 if not is_crossed else 72 # set loss
        pathloss_db = log_c0 + 10*alpha*math.log10(distance)
  
        ## received power including the gain from the number of antenna elements
        ant_element_num = 64 # number of antenna elements
        received_power = tx_power - pathloss_db + 10*math.log10(ant_element_num)
        
        ## check if the signal is detectable, i.e. its SNR
        ## exceeds the threshold (default setting, see `BSTransceiver.snr_threshold`)
        snr = received_power - BSTransceiver.noise
        if snr >= BSTransceiver.snr_threshold: 
            the_signal.quality = received_power
            the_signal.add_info("crossed_obstacle",is_crossed)
            return True # can detect
        else:
            the_signal.quality = None
            return False # cannot detect

class VehicleTransceiver(UserTransceiver):
    
    ## class constants or default values
    noise = -90 # dBm
    snr_threshold = 5 # dB

    def __init__(self, node, freq):
        super().__init__(node,freq)
        self.obstacle_list = None

    def create_signal(self):
        signal = super().create_signal()
        signal.tx_power = 20 # dBm
        return signal

    def set_obstacle(self, obstacle_list):
        self.obstacle_list = obstacle_list

    def is_crossed_obstacle(self, tx_loc, rx_loc) -> bool:
        if self.obstacle_list is None: return False
        point1 = tx_loc.get_xy()
        point2 = rx_loc.get_xy()
        line = [point1, point2]
        shapely_line = shapely.geometry.LineString(line)
        for polygon in self.obstacle_list:
            if(shapely_line.intersects(polygon)):
                return True
        

    def can_detect(self, the_signal) -> bool:
        '''This is the function to perform signal propagation and
        detection logic.'''

        ## retrieve all key info first
        tx_power = the_signal.tx_power      # get the tx power in dBm
        distance = the_signal.distance      # get the distance travelled
        travelling_dir = the_signal.LOS_dir # get the travelling direction
        tx_loc = the_signal.source._node.get("location") # sender location
        rx_loc = self._node.get("location")              # receiver location
        
        ## check if signal is crossing obstacle
        is_crossed = self.is_crossed_obstacle(tx_loc,rx_loc)

        ## apply simple passloss model c0*d^(-α), where
        #  - d is the distance, in our system, 2 pixels = 1m, minimum 1m
        #  - α is the pathloss exponent, 2 for normal, 2.92 when crossing an obstacle
        #  - c0 is the gain/loss
        ## pathloss PL_db = log10(c0) + 10α*log10(d)
        distance /= 2             # convert pixel-to-meter
        if distance<1: distance=1 # min 1m
        alpha = 2   # set pathloss exponent
        log_c0 = 61.4 # set loss
        pathloss_db = log_c0 + 10*alpha*math.log10(distance)
  
        ## received power including the gain from the number of antenna elements
        ant_element_num = 64 # number of antenna elements
        received_power = tx_power - pathloss_db + 10*math.log10(ant_element_num)
        ## check if the signal is detectable, i.e. its SNR
        ## exceeds the threshold (default setting, see `VehicleTransceiver.snr_threshold`)
        snr = received_power - VehicleTransceiver.noise if not is_crossed else 0
        if snr >= VehicleTransceiver.snr_threshold: 
            the_signal.quality = received_power
            the_signal.add_info("crossed_obstacle",is_crossed)
            return True # can detect
        else:
            the_signal.quality = None
            return False # cannot detect

####################################################################
## Nodes
####################################################################
        
class MySector(BaseNode):
    '''
    MySector: This is a sector of a base station in the VANET sim world
    '''
    def __init__(self, simworld, id, loc, carrier_freq, channel, sector_width, sector_dir):
        super().__init__(simworld, id, node_type=NodeType.BaseStation(self))

        self.transceiver = BSTransceiver(self,carrier_freq,channel,sector_width,sector_dir)

        ## setup the sector
        self.set_transceiver(self.transceiver)
        self.set_mobility(Stationary(loc))
        
        self.serving_node = None
        self.sector_id = 0
        self.connection_time = 0.0
        self.connection_list = []

    ## show the coverage of this sector
    def show_coverage(self):
        self.clear_drawing() # this is persistent drawing, so need to clear the all first
        if self.serving_node!=None:
            if self.transceiver.get_property("type")=="omni":
                self.draw_circle(self.transceiver.get_property("radius"))
            elif (self.transceiver.get_property("type")=="directional"
                or self.transceiver.get_property("type")=="mmWaveBeam"):
                self.draw_sector(self.transceiver.get_property("radius"),
                                 self.transceiver.get_property("azimuth"),
                                 self.transceiver.get_property("beam width"))

class MyVehicle(BaseNode):
    '''
    MyVehicle: This is a transmitting node in the VANET sim world implementing 
    a user-defined transceiver.
    '''
    def __init__(self, simworld, id, map, carrier_freq):
        super().__init__(simworld, id, node_type=NodeType.Vehicle(self))

        ## initialize some properties
        self.transceiver = VehicleTransceiver(self,carrier_freq)
        self.set_transceiver(self.transceiver)
        self.map = map
        
        self.location = None
        self.direction = None
        self.distance_to_BS = None
        self.relay_direction = None
        self.associated_sector = None
        self.connection_quality = None
        self.relay_quality = None
        self.crossed_obstacle = False
        self.associated_vehicle = None
        self.ready_to_connect = False
        self.serving_node = None
        self.connection_time = 0.0
        self.disconnection_time = 0.0
        self.time_connected = 0.0
        self.context = []
        
    def set_route(self, start_pin, end_pin, speed_low, speed_high) -> bool:
        path = self.map.find_path(start_pin,end_pin)
        if len(path)==0: return False # can't find a path

        route = []
        x,y,_ = path[0] # unpack the starting point
        start_loc = XY(x,y) # create an XY point
        speed = random.uniform(speed_low,speed_high)   # in km/h
        speed = self.map.kph(speed, speed_up_factor=5) # in pixels/sec
        for waypoint in path[1:]:
            route.append((speed,XY(waypoint[0],waypoint[1])))
        self.set_mobility(StaticPath(start_loc,route))
        self.end_pin = end_pin
        return True

    ## draw a line to the connected BS, if any
    def show_connection(self):
        self.clear_drawing()
        
        if self.associated_sector is not None:
            self.draw_text(10,15,"%1.2fdBm"%self.connection_quality, wx.Font(10, wx.FONTFAMILY_ROMAN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))
            if self.crossed_obstacle:
                self.draw_line(self.associated_sector,pen = wx.Pen(wx.BLUE,1,style=wx.PENSTYLE_SHORT_DASH))
                self.set_color(wx.BLUE)
            else:
                self.draw_line(self.associated_sector,pen = wx.Pen(wx.BLACK,2,style=wx.PENSTYLE_SOLID))
                self.set_color(wx.BLACK)
                
        elif self.associated_sector is None and self.associated_vehicle is not None:
            self.draw_text(10,15,"%1.2fdBm"%self.relay_quality, wx.Font(10, wx.FONTFAMILY_ROMAN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL))  
            if self.crossed_obstacle:
                self.draw_line(self.associated_vehicle,pen = wx.Pen(wx.BLUE,1,style=wx.PENSTYLE_SHORT_DASH))
                self.set_color(wx.BLUE)
            else:
                self.draw_line(self.associated_vehicle,pen = wx.Pen(wx.GREEN,2,style=wx.PENSTYLE_SOLID))
                self.set_color(wx.GREEN)
                
        else:
            self.set_color(wx.RED)
            

####################################################################
## Scenario
####################################################################

class MyScenario(BaseScenario):
    '''
    MyScenario: This is my scenario.
    '''
    ##---------------------------------------------------------------
    ## This method will be called before the start of the simulation,
    ## build the simulation world here
    def on_create(self, simworld) -> bool:
        
        global sector_list
        
        ### load the map image and info
        map = MapInfo()
        map.load_file(image_file="Guildford-UK-3.png",  # map file used, with the required obstacles, stations and parkings
                      data_file="Guildford-UK-3.json")
        if not map.is_ready():
            print("Failed to load the map. The reason is: %s\n"%map.get_err_str())
            return False

        ## use the map and give a name to this scenario
        self.use_map(map)
        self.set_name("Vehicle Mobility in Guildford, Surrey, UK")
        
        ## set up CMAB
        self.cmab = CMAB()

        ## read obstacles from map file and draw them in simulation    
        obstacles = {}
        for i in range(0,62):   # hard coded, must be equal to the number of obstacles
            string = "OBS-%i" %(i+1)
            obstacles[string] = None
                      
        for pin_name in obstacles:
            my_obstacle = Pin.Region(map, pin_name)
            obs_loc = my_obstacle.get_xy()
            obs_vertices = my_obstacle.get_vertices()
            vertices_list = []
            for i in range(len(obs_vertices)):
                for j in range(len(obs_vertices[i])):
                    vertices_list.append(obs_vertices[i][j])
            vertices_list = [vertices_list[i:i + 2] for i in range(0, len(vertices_list), 2)]
            for k in range(len(vertices_list)):
                vertices_list[k][0] = vertices_list[k][0]+obs_loc[0]
                vertices_list[k][1] = vertices_list[k][1]+obs_loc[1]
            obstacles[pin_name] = vertices_list
            if obstacles[pin_name]==None:
                print("Failed to load this pin: %s."%pin_name)
                return False
        
        self.all_obstacles = []
        for pin_name in obstacles:
            obstacle = obstacles[pin_name]
            shapely_poly = shapely.geometry.Polygon(obstacle)
            self.add_drawable(Drawable().polygon(obstacle).set_drawing(wx.BLUE_PEN,wx.YELLOW_BRUSH))
            self.all_obstacles.append(shapely_poly)

        ## load some pin location info from the map to place the BSs
        bs_loc = { "BS-1": None}    # hard coded, must match the ids of the Base Stations
        for pin_name in bs_loc:
            bs_loc[pin_name] = map.get_pin_xy(pin_name)
            if bs_loc[pin_name]==None:
                print("Failed to load this pin: %s."%pin_name)
                return False
                
        ## create BSs at those loaded locations
        carrier_freq = 2.4
        ch_omni = DiscModel(radius=map.km(0.3))
        self.all_sectors = []
        beam_num = 16     # must be an integer
        sector_width = 360/beam_num
        beam_pointing = 0 # 0 means north
        sector_dir = []
        for i in range(0,beam_num):
            angle = beam_pointing + i*sector_width
            while angle>=360: angle-=360
            sector_dir.append(angle)
        
        for pin_name in bs_loc:
            for j in range(0,beam_num): # sectors for each BS
                new_sector = MySector(simworld, pin_name, XY(xy=bs_loc[pin_name]), carrier_freq, ch_omni, sector_width, sector_dir[j])
                new_sector.sector_id = j
                self.all_sectors.append(new_sector)
                sector_list.append(new_sector)
                new_sector.transceiver.set_obstacle(self.all_obstacles)

        ## create some vehicles
        self.vehicles = []
        self.parking = [ "P1", "P2",    # hard coded, names must match those of the parkings in the map
                         "P3", "P4", 
                         "P5", "P6",
                         "P7", "P8",
                         "P9", "P10"]
        self.vehicle_info = {}
        global num_vehicles
        for l in range(num_vehicles):
            this_vehicle = MyVehicle(simworld, "Car %d"%l, map, carrier_freq)
            while True:
                start = random.choice(self.parking)
                end = random.choice(self.parking)
                if end==start: continue
                if this_vehicle.set_route(start, end, speed_low=20, speed_high=40):
                    break
            self.vehicles.append(this_vehicle)
            this_vehicle.transceiver.set_obstacle(self.all_obstacles)

        return True

    ##-------------------------------------------------------------
    ## This method will be called repeatedly until the simulation
    ## is ended or stopped, perform any user simulation action here
    
    def on_event(self, sim_time, event_obj):
        if event_obj==Event.MOBILITY_END: # a mobile node has finished its mobility?
            self.do_restart_node(sim_time,event_obj)
        elif event_obj==Event.SIM_MOBILITY: # mobility progresses a time step?
            self.do_mobility(sim_time,event_obj)
    
    def do_restart_node(self, sim_time, event_obj):
        this_node = event_obj.get("node") # get the node reaching end of mobility
        start = this_node.end_pin    # this will be the start location
        while True:
            end = random.choice(self.parking)  # end location is a random choice
            if end==start: continue
            if this_node.set_route(start, end, speed_low=20, speed_high=40):
                break # route successful established, break
    
    def do_mobility(self, sim_time, event_obj):
        
        global active_beams                    
        global connection_time_per_second
        global connection_time_list
        global arm_connection_list
        global counter
        global sim_duration
        
        if (sim_time<0.2*sim_duration):
            for this_sector in self.all_sectors:
                if active_beams >= 4: continue   # skip if active beams are above maximum
                beacon = this_sector.transceiver.create_signal()
                reply_list = this_sector.transceiver.broadcast(beacon)
                
                for (vehicle,signal) in reply_list:
                    if this_sector.serving_node!=None: continue
                    if vehicle.type!=NodeType.Vehicle: continue
                    if vehicle.associated_sector!=None: continue
                    
                    beacon_reply = vehicle.transceiver.create_signal()
                    recv_signal = vehicle.transceiver.unicast(beacon_reply,this_sector)

                    if recv_signal is None: continue # skip if failed, likely not in coverage
                    
                    ## Vehicle connected to sector ##
                    vehicle.associated_sector = this_sector
                    this_sector.serving_node = vehicle
                    vehicle.connection_quality = recv_signal.quality
                    vehicle.crossed_obstacle = recv_signal.get_info("crossed_obstacle")
                    active_beams = active_beams + 1
                    vehicle.connection_time = sim_time

        else:
            sectors_ranked = []

            for this_sector in self.all_sectors:
                beacon = this_sector.transceiver.create_signal()
                reply_list = this_sector.transceiver.broadcast(beacon)
                
                for (vehicle,signal) in reply_list:
                    if vehicle.type!=NodeType.Vehicle: continue
                    vehicle_direction_reward = self.cmab.get_reward(vehicle.direction, this_sector.sector_id)
                    vehicle_distance_reward = self.cmab.get_reward(vehicle.distance_to_BS, this_sector.sector_id)
                    vehicle_relay_direction_reward = self.cmab.get_reward(vehicle.relay_direction, this_sector.sector_id)
                    sector_reward = [this_sector, vehicle_direction_reward*1.5 + vehicle_distance_reward + vehicle_relay_direction_reward] # higher weighting to directions
                    sectors_ranked.append(sector_reward)
            
            sectors_ranked.sort(key=operator.itemgetter(1), reverse=True)
                
            for (this_sector, reward) in sectors_ranked:
                if active_beams >= 4: continue   # skip if active beams are above maximum
                beacon = this_sector.transceiver.create_signal()
                reply_list = this_sector.transceiver.broadcast(beacon)
                
                for (vehicle,signal) in reply_list:
                    if this_sector.serving_node!=None: continue
                    if vehicle.type!=NodeType.Vehicle: continue
                    if vehicle.associated_sector!=None: continue
                    
                    beacon_reply = vehicle.transceiver.create_signal()
                    recv_signal = vehicle.transceiver.unicast(beacon_reply,this_sector)

                    if recv_signal is None: continue # skip if failed, likely not in coverage
                    
                    ## Vehicle connected to sector ##
                    vehicle.associated_sector = this_sector
                    this_sector.serving_node = vehicle
                    vehicle.connection_quality = recv_signal.quality
                    vehicle.crossed_obstacle = recv_signal.get_info("crossed_obstacle")
                    active_beams = active_beams + 1
                    vehicle.connection_time = sim_time
                    
        ## vehicle-to-sector disconnection    
        for this_sector in self.all_sectors:
            vehicle = this_sector.serving_node
            if vehicle == None: continue # skip if vehicle is not connected to a sector
            
            beacon = vehicle.transceiver.create_signal()
            signal = vehicle.transceiver.unicast(beacon,this_sector)
            
            if signal is None: # vehicle lost connection with relay vehicle

                ## update rewards after disconnection
                vehicle.disconnection_time = sim_time
                vehicle.time_connected = vehicle.disconnection_time - vehicle.connection_time
                this_sector.connection_time = this_sector.connection_time + vehicle.time_connected
                self.cmab.update_reward(vehicle.direction, this_sector.sector_id, vehicle.time_connected)
                self.cmab.update_reward(vehicle.distance_to_BS, this_sector.sector_id, vehicle.time_connected)
                self.cmab.update_reward(vehicle.relay_direction, this_sector.sector_id, vehicle.time_connected)
                connection_time_per_second = connection_time_per_second + vehicle.time_connected
                
                ## do disconnection
                active_beams = active_beams - 1
                this_sector.serving_node = None
                vehicle.associated_sector = None
                vehicle.connection_quality = None

        ## vehicle-to-vehicle connection
        for vehicle in self.vehicles:
            if vehicle.associated_sector==None: continue # skip if vehicle is not connected to a sector
            beacon = vehicle.transceiver.create_signal()
            reply_list = vehicle.transceiver.broadcast(beacon)
            
            for (relay_vehicle,signal) in reply_list:
                if vehicle.serving_node!=None: continue
                if relay_vehicle.type!=NodeType.Vehicle: continue # skip if not a vehicle type
                if relay_vehicle.associated_vehicle!=None: continue #skip if vehicle is already serving other  
                
                beacon_reply = relay_vehicle.transceiver.create_signal()
                relay_signal = relay_vehicle.transceiver.unicast(beacon_reply,vehicle)
                if relay_signal is None: continue # skip if failed, likely not in coverage
                
                ## Vehicle connected to relay vehicle ##
                relay_vehicle.associated_vehicle = vehicle
                vehicle.serving_node = relay_vehicle
                relay_vehicle.relay_quality = relay_signal.quality
                relay_vehicle.crossed_obstacle = relay_signal.info["crossed_obstacle"]
                relay_vehicle.connection_time = sim_time
        
        ## vehicle-to-vehicle disconnection
        for relay_vehicle in self.vehicles:
            vehicle = relay_vehicle.associated_vehicle
            if vehicle == None: continue
            vehicle_sector = vehicle.associated_sector

            beacon = relay_vehicle.transceiver.create_signal()
            signal = relay_vehicle.transceiver.unicast(beacon,vehicle)
            
            if signal is None: # vehicle lost connection with relay vehicle
                relay_vehicle.disconnection_time = sim_time
                relay_vehicle.time_connected = relay_vehicle.disconnection_time - relay_vehicle.connection_time
                vehicle.time_connected = vehicle.time_connected + relay_vehicle.time_connected*1.2
                vehicle.serving_node = None
                relay_vehicle.associated_vehicle = None
                relay_vehicle.relay_quality = None
            
            if vehicle_sector is None: # relay vehicle lost connection with sector, so vehicle loses connection with relay vehicle
                relay_vehicle.disconnection_time = sim_time
                relay_vehicle.time_connected = relay_vehicle.disconnection_time - relay_vehicle.connection_time
                vehicle.time_connected = vehicle.time_connected + relay_vehicle.time_connected*1.2
                vehicle.serving_node = None
                relay_vehicle.associated_vehicle = None
                relay_vehicle.relay_quality = None
                
        ## determine context of vehicles
        for vehicle in self.vehicles:
            context = []
            
            ## direction, can be 8: N, NE, E, SE, S, SW, W, NW
            direction = vehicle.get("direction").get_azimuth()
            if direction >=0 and direction < 22.5 or direction >= 337.5 and direction <= 360:
                vehicle.direction = "N"
            elif direction >= 22.5 and direction < 67.5:
                vehicle.direction = "NE"
            elif direction >= 67.5 and direction < 112.5:
                vehicle.direction = "E"
            elif direction >= 112.5 and direction < 157.5:
                vehicle.direction = "SE"
            elif direction >= 157.5 and direction < 202.5:
                vehicle.direction = "S"
            elif direction >= 202.5 and direction < 247.5:
                vehicle.direction = "SW"
            elif direction >= 247.5 and direction < 292.5:
                vehicle.direction = "W"
            elif direction >= 292.5 and direction < 337.5:
                vehicle.direction = "NW"
            context.append(vehicle.direction)
            
            ## distance from BS
            sector = self.all_sectors[0]
            distance = vehicle.get("location").distance_to(sector.get("location"))
            if 0 < distance < 200:
                vehicle.distance_to_BS = "Near"
            elif 200 <= distance < 400:
                vehicle.distance_to_BS = "Middle"
            elif distance >= 400:
                vehicle.distance_to_BS = "Far"
            context.append(vehicle.distance_to_BS)
            
            ## if vehicle is relaying to another vehicle, direction of relay vehicle
            if vehicle.serving_node!=None:
                vehicle.relay_direction = vehicle.serving_node.direction
                context.append(vehicle.relay_direction)
                
            vehicle.context = context

        ## show connections
        for vehicle in self.vehicles:
            vehicle.show_connection()
        for sector in self.all_sectors:
            sector.show_coverage()
            
        ## calculate mean connection time every 15 seconds
        counter = counter + 1.0
        if (counter >= 1800.0):
            connection_time_list.append(connection_time_per_second/180)
            connection_time_per_second = 0.0
            counter = 0.0
            sector_connection_list=[]
            for sector in self.all_sectors:
                sector.connection_list.append(sector.connection_time)
                sector.connection_time = 0.0
                
######################################################################
## Multi-Armed Bandit & Contextual MAB
######################################################################
class MAB:
    def __init__(self):
        self.total_rewards = {}
        self.total_count = {}
        self.average_reward = {}

    def update_reward(self, arm, reward):
        if arm not in self.total_rewards: 
            self.total_rewards[arm] = 0
            self.total_count[arm] = 0
        self.total_rewards[arm] += reward
        self.total_count[arm] += 1
        self.average_reward[arm] = self.total_rewards[arm]/self.total_count[arm]

    def get_reward(self, arm):
        if arm not in self.average_reward: return 0
        return self.average_reward[arm]

    def get_best_arm(self): # return a tuple (arm,reward)
        return max(self.average_reward.items(), key=operator.itemgetter(1))
               
class CMAB:
    def __init__(self):
        self.mab = {}

    def update_reward(self, context, arm, reward):
        if context not in self.mab: self.mab[context] = MAB()
        self.mab[context].update_reward(arm, reward)

    def get_reward(self, context, arm):
        if context not in self.mab: return 0
        return self.mab[context].get_reward(arm)

    def get_best_arm(self, context):
        if context not in self.mab: return None
        return self.mab[context].get_best_arm()

####################################################################
## main
####################################################################

if __name__ == "__main__":

    ## create, setup and run the simulation
    ## note that to run a simulation, we need to create a 'scenario'
    run_flag = True
    while run_flag:
        sim = World()
        sim.config(sim_stop=sim_duration, 
                sim_step=0.1, 
                sim_speed=1.0, 
                display_option=True, #change to False to disable animation
                scenario=MyScenario(sim))
        run_flag = sim.run()
   
    # graph 1, mean connection time of all vehicles
    plt.plot(connection_time_list)
    plt.xlabel("Simulation Time")
    plt.ylabel("Mean Connection Time")
    plt.savefig("mean_connection_time.png")
    plt.show()
    
    # graph 2, mean connection time of all vehicles in each arm
    sim_time_list = []
    for sector in sector_list:
        arm_connection_list.append(sector.connection_list)
    plt.xlabel("Simulation Time")
    plt.ylabel("Arm Mean Connection Time")
    for i in range(len(arm_connection_list)):
        plt.plot(arm_connection_list[i],label = 'Arm %s'%(i+1))
    plt.legend()
    plt.savefig("arm_mean_connection_time.png")
    plt.show()
    plt.close()