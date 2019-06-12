# -*- coding: utf-8 -*-
"""
Created on Thu Oct  4 15:08:37 2018

@author: mat_v
"""

import numpy as np
from math import *
from enum import Enum

class DataPointType(Enum):
    IMU = 1
    GPS = 2


"""
 * @brief Data interface for getting fusion actionable data from raw sensor measurements
"""
class DataPoint:
    """
     * @brief Default constructor
    """
    def __init__(self):
        self._initialized = False
        self._first_data_point = True
        self._dx = 0
        self._dy = 0
        self._mx = 0
        self._my = 0
        self._ds = 0
        self._last_psi = 0
        
        self._RadiusEarth = 6378388.0 # [m]
        self._arc = 2.0 * np.pi * (self._RadiusEarth + 230)/360.0 # [degree]

        #self.GPSDataConverter = GeodecticConverter()

    """
     * @brief Retrieves raw sensor data and stores it in private variables
     *
     * @param timestamp Current timestamp for the sensor data
     * @param data_type Data type: Either GPS: which includes GPS+IMU data or IMU: which only includes the IMU data
     * @param raw_data Raw sensor data
    """
    
    def data_set(self, timestamp, data_type, raw_data):
        #np.resize(self._raw_data, raw_data.shape);
        self._timestamp = timestamp
        self._data_type = data_type;
        self._raw_data = raw_data
        self._initialized = True

        if(self._first_data_point and raw_data[0,0]!=0.0 and raw_data[1,0]!=0.0):
            self._dx = 0.0
            self._dy = 0.0
            self._mx = 0.0
            self._my = 0.0
            self._prev_lat = raw_data[0,0]
            self._prev_long = raw_data[1,0]
            self._arc = 2.0 * np.pi * (self._RadiusEarth + raw_data[6,0])/360.0
            self._first_data_point = False
        elif(not self._first_data_point):
            self._arc = 2.0 * np.pi * (self._RadiusEarth + raw_data[6,0])/360.0
            self._dx = self._arc * cos(raw_data[0,0] * np.pi/180.0) * (raw_data[1,0] - self._prev_long)
            self._dy = self._arc * (raw_data[0,0] - self._prev_lat)
            self._ds = sqrt(self._dx**2 + self._dy**2)
            
            if(self._ds == 0.0):
                self._data_type = DataPointType.IMU
            else:
                self._data_type = DataPointType.GPS
                
            self._mx += self._dx     # cumulative sum
            self._my += self._dy     # cumulative sum
            
            self._prev_lat = raw_data[0,0]
            self._prev_long = raw_data[1,0]
            
        # Checks for discontinuity in psi
        if(abs(self._raw_data[2,0]-self._last_psi) > pi):
            self._raw_data[2,0] -= np.sign(self._raw_data[2,0]-self._last_psi)*2*pi
            
        self._last_psi = self._raw_data[2,0]
        
    """
     * @brief Returns saved raw data for sensor fusion
     *
     * @return Sensor data measurements
    """
    def get_state(self):
        #state = np.empty([6,1])

        # Convert raw data to fusion readable states
        x = self._mx
        y = self._my
        psi = self._raw_data[2,0]
        vel = self._raw_data[3,0]
        psi_dot = self._raw_data[4,0]
        a = self._raw_data[5,0]
        b = 0.0
        
        state = np.array([[x], [y], [psi], [vel], [psi_dot], [a], [b]])

        return state;

    """
     * @brief Get raw sensor data
     *
     * @return Raw sensor data
    """
    def get_raw_data(self):
        return self._raw_data

    """
     * @brief Get data type associated with the data at current timestep
     *
     * @return Data type: Either GPS or IMU
    """
    def get_data_point_type(self):
        return self._data_type

    """
     * @brief Get current timestamp
     *
     * @return Timestamp associated with current data
    """
    def get_timestamp(self):
        return self._timestamp