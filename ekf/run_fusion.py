# -*- coding: utf-8 -*-
"""
Created on Thu Oct  4 13:57:05 2018

@author: Mateus Valverde

This is the run_fusion.cpp used in GPS_INS algorithm, implemented in python by Mateus

Main Class running GPS INS Fusion

Created by Karan on 5/2/18.
Copyright 2018 Karan. All rights reserved. 
"""

"""
 * @brief Constructor for the main class
 *
 * @param max_acceleration maximum acceleration for the system
 * @param max_turn_rate maximum turn rate for the system
 * @param max_yaw_accel maximum yaw acceleration for the system
 * These parameters are used to set up the system noise matrices in the EKF
"""

from ekf.fusion import *
from ekf.DataPoint import *
from ekf.paramReader import *

class GpsIns:
    def __init__(self):
        paramReader = ParameterReader()
        
        varGPS = paramReader.getData("vargps")
        varHeading = paramReader.getData("varheading")
        varSpeed = paramReader.getData("varspeed")
        varYaw = paramReader.getData("varyaw")
        varAcc = paramReader.getData("varaccel")
        maxAccel = paramReader.getData("maxaccel")
        maxTurnRate = paramReader.getData("maxturnrate")
        maxYawAccel = paramReader.getData("maxyawaccel")
        xOff = paramReader.getData("xOff")
        yOff = paramReader.getData("yOff")
        
        self.filter = Fusion(maxAccel, maxTurnRate, maxYawAccel, varGPS, varHeading, varSpeed, varYaw, varAcc, xOff, yOff)
                
        self._raw_data = np.zeros([7,1])
        self._prev_enc_counter = 0.0
        self._prev_gps_counter = 0.0
        self._prev_imu_counter = 0.0
        self._imucounter = 0.0
        self._gpscounter = 0.0
        self._enccounter = 0.0
        self._dt = 0.0
        self._prev_time = 0.0
        self._sensor_data = DataPoint()

    # @brief Reads in the GPS data
    
    def read_gps_data(self, lat, lon, alt):
        print("In read GPS")
    #    m.lock();
        self._raw_data[0,0] = lat
        self._raw_data[1,0] = lon
        self._raw_data[6,0] = alt
        self._gpscounter += 1
    #    m.unlock();
    
    """
     * @brief Reads IMU values
     *
     * @param yaw_rate psi_dot
     * @param long_accel longitudinal acceleration of the robot
     * @param timestamp current timestamp
    """
    def read_imu_data(self, heading, yaw_rate, long_accel):
        #m.lock()
        self._raw_data[2,0] = heading
        self._raw_data[4,0] = yaw_rate
        self._raw_data[5,0] = long_accel
        self._imucounter += 1
        #m.unlock()
    
    """
     * @brief Reads in the encoders - called in tasks
     *
     * @param psi yaw values computed from encoders
     * @param vel velocity computed from encoders
    """
    def read_encoders(self, vel):
        print("In encoder read")
        #m.lock()
        self._raw_data[3,0] = vel
        self._enccounter += 1
        #m.unlock()
    
    """
     * @brief Sets up the data in DataPoint class to be used up by the filter
     * How do you ensure data in this object is the current and has been completely filled at this timestep?
    """
    def set_data(self):
        print("Setting data")
    
        flag = (self._gpscounter > self._prev_gps_counter)
    
        # iF gps got updated then we update the filter else we just predict
        if(flag):
            self._data_type = DataPointType.GPS
        else:
            self._data_type = DataPointType.IMU
        # Assuming constant timeinterval - possible cause of values blowing up
        self._sensor_data.data_set(self._dt, self._data_type, self._raw_data)
    
        print("Data set")
    
    """
     * @brief Main loop for the filter
    """
    def loop(self, ts):
        
        cur_time = ts*1.e6
        self._dt = cur_time - self._prev_time
        # update previous time only if dt > 0
        if(self._dt > 0.0): self._prev_time = cur_time
        
        self.set_data()
        self.filter.process(self._sensor_data)
        #m.lock()
        self._prev_gps_counter = self._gpscounter
        #m.unlock()
    
    """
     * @brief Returns the estimated state of the system
     *
     * @return Estimated state for the robot
    """
    def get_estimated_state(self):
        return self.filter.get_resulting_state();
    
    def get_sensor_data(self):
        return self._sensor_data;
