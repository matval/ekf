# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 18:32:46 2018

@author: mat_v
"""
# Clear all variables
from IPython import get_ipython
get_ipython().magic('reset -sf')

import matplotlib.pyplot as plt
import numpy as np
from math import *
from ekf.run_fusion import *

import sys
sys.path.append('../Datalog Import Data')

from TerraSentia import *
from import_data_fcn import *
from gps2meters import *

# Close all figures
plt.close("all")

path = 'C:\\Users\\mat_v\\Desktop\\collections\\190117 - simulation'

exp_robot = TerraSentia(path)
[exps, pTS, datalog, lidar] = import_data_fcn(path)

# Print the experiments available in the path location
print('Experiments available:')
print(exp_robot.GetExperiments())
 
# Choose the index of the experiment you want to use
exp_robot.SetExperiment(1)
exp = exps[1]

############### Kalman Filter Matrices Definitions ############################

# Data read from datalog:
rt_est_x    = datalog[exp].estimated_state0
rt_est_x    = rt_est_x.astype(np.float)
rt_est_x   -= float(rt_est_x[0])
rt_est_y    = datalog[exp].estimated_state1
rt_est_y    = rt_est_y.astype(np.float)
rt_est_y   -= float(rt_est_y[0])
rt_est_phi  = datalog[exp].estimated_state2
rt_est_v    = datalog[exp].estimated_state3
rt_est_dphi = datalog[exp].estimated_state4
rt_est_a    = datalog[exp].estimated_state5
rt_est_b    = datalog[exp].estimated_state6

gps_lat = datalog[exp].primary_gps_latitude_deg
gps_long = datalog[exp].primary_gps_longitude_deg
gps_lat2 = datalog[exp].secondary_gps_latitude_deg
gps_long2 = datalog[exp].secondary_gps_longitude_deg
gps_alt = np.zeros((1,len(gps_lat2)))
gps2_data = np.zeros((3, len(gps_lat2)))
gps2_data[0,:] = gps_lat2
gps2_data[1,:] = gps_long2

gps_parser = GPS2Meters([gps_lat[0], gps_long[0]])
gps_parser.data_set(gps2_data)
gps2_meters = gps_parser.get_positions()

imu_gyro_yaw_rate = -datalog[exp].gyro_yaw_rate_rad_s
imu_yaw_angle = datalog[exp].yaw_rad

input_mhe_x = datalog[exp].input_x_for_mhe_m
input_mhe_y = datalog[exp].input_y_for_mhe_m
output_mhe_x = datalog[exp].mhe_output_x_m
output_mhe_y = datalog[exp].mhe_output_y_m
output_mhe_yaw = datalog[exp].mhe_output_yaw_rad

mag_heading = datalog[exp].mag_heading

accel_x = datalog[exp].accelerometer_x_m_s2

datalog_ts  = datalog[exp].uptime_ms/1000
datalog_ts -= datalog_ts[0]

# Initialization of state matrices
est_x = np.array([])
est_y = np.array([])
gps_x = np.array([])
gps_y = np.array([])
est_heading = np.array([])
est_speed = np.array([])
est_yaw_rate = np.array([])
est_accel = np.array([])
est_bias = np.array([])
lidar_heading = np.array([])
gps_heading = np.array([])
imu_yaw_rate = np.array([])
perc_yaw_rate = np.array([])
odom_speed = np.array([])
odom_accel = np.array([])
imu_accel = np.array([])
right_row_x = np.array([])
right_row_y = np.array([])
left_row_x = np.array([])
left_row_y = np.array([])
time = np.array([])

fusion = GpsIns()

dt = 0.005
ts = 0.001
dist1 = 0
dist2 = 0
dist3 = 0
last_psi = 0
last_speed = 0
last_gps_x = 0
last_gps_y = 0
last_est_x = 0
last_est_y = 0
last_heading = 0
accel = np.zeros(1)
yaw_rate = np.zeros(1)
while(exp_robot.NotFinished(ts)):
    # Getting the sensors readings
    
    if(exp_robot.GetRawGPS()):
        Raw_GPS = exp_robot.GetRawGPS()
        fusion.read_gps_data(Raw_GPS[0], Raw_GPS[1], 0.0)
    '''
    if(exp_robot.GetSecondaryRawGPS()):
        Raw_GPS = exp_robot.GetSecondaryRawGPS()
        print("GPS data: %.6f", Raw_GPS)
        fusion.read_gps_data(Raw_GPS[0], Raw_GPS[1], 0.0)
    '''
    if(exp_robot.GetOdometry()):
        Encoder_Vel = (exp_robot.GetOdometry()[0] + exp_robot.GetOdometry()[1])/2
        last_speed = Encoder_Vel
        fusion.read_encoders(Encoder_Vel)
    if(exp_robot.GetRawIMU()):
        Raw_IMU = exp_robot.GetRawIMU()
        yaw_angle = exp_robot.GetRPY()
        yaw_rate = np.roll(yaw_rate,1)
        yaw_rate[0] = -Raw_IMU.gyro_reading[2]
        accel = np.roll(accel,1)
        accel[0] = Raw_IMU.accel_reading[0]
        fusion.read_imu_data(yaw_angle[2], np.mean(yaw_rate), np.mean(accel))
    '''
    if(exp_robot.GetPerception()):
        Perception_Psi = exp_robot.GetPerception()[0]
        right_row_d = exp_robot.GetPerception()[3]
        left_row_d = exp_robot.GetPerception()[2]
    '''
    exp_robot.Loop(ts)
        
    fusion.loop(ts)
    result2 = fusion.get_estimated_state()
    result = fusion.get_sensor_data()
    raw_data = result.get_state()

    ts += dt
    
    ############################
    
    # Kalman output
    gps_x = np.append(gps_x, raw_data[0,0])
    gps_y = np.append(gps_y, raw_data[1,0])
    est_x = np.append(est_x, result2[0,0])
    est_y = np.append(est_y, result2[1,0])
    est_heading = np.append(est_heading, result2[2,0])
    est_speed = np.append(est_speed, result2[3,0])
    est_yaw_rate = np.append(est_yaw_rate, result2[4,0])
    est_accel = np.append(est_accel, result2[5,0])
    est_bias = np.append(est_bias, result2[6,0])
    time = np.append(time, ts)
    '''
    # Corn rows:
    right_row_x = np.append(right_row_x, result2[0,0] + right_row_d*np.sin(result2[2,0]))
    right_row_y = np.append(right_row_y, result2[1,0] - right_row_d*np.cos(result2[2,0]))
    left_row_x = np.append(left_row_x, result2[0,0] - left_row_d*np.sin(result2[2,0]))
    left_row_y = np.append(left_row_y, result2[1,0] + left_row_d*np.cos(result2[2,0]))
    '''
    # Estimating the heading using GPS to compare to the results
    if(raw_data[1,0] != last_gps_y or raw_data[0,0] != last_gps_x):
        new_gps_heading = atan2(raw_data[1,0]-last_gps_y,raw_data[0,0]-last_gps_x)
        if(abs(new_gps_heading-last_heading) > pi):
            new_gps_heading -= np.sign(new_gps_heading-last_heading)*2*pi
        gps_heading = np.append(gps_heading, new_gps_heading)
        last_heading = gps_heading[-1]
    else:
        gps_heading = np.append(gps_heading, last_heading)
    
    if(ts != 0.001):
        dist1 += sqrt((result2[0,0]-last_est_x)**2 + (result2[1,0]-last_est_y)**2)
    
    last_est_x = result2[0,0]
    last_est_y = result2[1,0]
    
    dist2 += est_speed[-1]*dt

    #lidar_heading = np.append(lidar_heading, Perception_Psi)
    imu_yaw_rate = np.append(imu_yaw_rate, np.mean(yaw_rate))
    imu_accel = np.append(imu_accel, np.mean(accel))
    odom_speed = np.append(odom_speed, Encoder_Vel)
    
    last_gps_x = raw_data[0,0]
    last_gps_y = raw_data[1,0]

# Calculation of total traveled length
for i in range(1, len(rt_est_x)):
    dist3 += sqrt((rt_est_x[i]-rt_est_x[i-1])**2 + (rt_est_y[i]-rt_est_y[i-1])**2)
    
print("Traveled length according to python script: ", dist1)
print("Traveled length according to speed integration: ", dist2)
print("Traveled length according to real time code: ", dist3)

######################### Plots ###############################################
f, axarr = plt.subplots(2, 3)
axarr[0, 0].set_title('Localization')
axarr[0, 0].scatter(gps_x, gps_y, marker=".", color='r', label='Primary GPS')
axarr[0, 0].scatter(gps2_meters[0,:], gps2_meters[1,:], marker=".", color='orange', label='Secondary GPS')
axarr[0, 0].plot(-(output_mhe_x-output_mhe_x[6]), output_mhe_y-output_mhe_y[6], 'b', label='MHE localization')
#axarr[0, 0].plot(est_x, est_y, 'r', label='GPS_INS in Python')
axarr[0, 0].plot(rt_est_x, rt_est_y, 'y', label='real Time EKF')
#plt.plot(right_row_x, right_row_y, '--g', label='right row')
#plt.plot(left_row_x, left_row_y, '--g', label='left row')
axarr[0, 0].set_xlabel("x [m]")
axarr[0, 0].set_ylabel("y [m]")
# Put legend
axarr[0, 0].legend(loc='upper left')

axarr[0, 1].set_title('Heading Estimation')
axarr[0, 1].plot(time, gps_heading, 'purple', label='Heading by GPS')
axarr[0, 1].plot(datalog_ts, -output_mhe_yaw-pi, 'b', label='MHE Yaw Angle')
axarr[0, 1].plot(datalog_ts, imu_yaw_angle, 'g', label='IMU Yaw Angle')
axarr[0, 1].plot(time, est_heading, 'r', label='Heading with GPS_INS in Python')
axarr[0, 1].plot(datalog_ts, rt_est_phi, 'y', label='Heading with Real Time GPS_INS')
axarr[0, 1].plot(datalog_ts,mag_heading, 'orange', label='Magnetometer Heading')
#axarr[0, 1].plot(time, lidar_heading, 'b', label='Heading estimated by Perception')
#axarr[0, 1].set_ylim(-10, 10)
axarr[0, 1].set_xlabel("time [s]")
axarr[0, 1].set_ylabel("heading [rad]")
# Put legend
axarr[0, 1].legend(loc='upper left')

'''
######## Raw GPS Plots comparing primary and secondary GPS points #############
axarr[0, 2].set_title('Localization')
axarr[0, 2].plot(gps_long, gps_lat, '.b', label='Raw Primary GPS')
axarr[0, 2].plot(gps_long, gps_lat2, '.r', label='Raw Secondary GPS')
axarr[0, 2].set_xlabel("longitude")
axarr[0, 2].set_ylabel("latitude")
# Put legend
axarr[0, 2].legend(loc='upper left')
'''
######## Bias Plots #############
axarr[0, 2].set_title('Bias')
axarr[0, 2].plot(time, est_bias, 'r', label='Bias by Python')
axarr[0, 2].plot(datalog_ts, rt_est_b, 'y', label='Real Time Bias')
axarr[0, 2].set_xlabel("time [s]")
axarr[0, 2].set_ylabel("bias [rad]")
# Put legend
axarr[0, 2].legend(loc='upper left')

#####################################################################
axarr[1, 2].plot(datalog_ts, imu_gyro_yaw_rate, 'g', label='Datalog Yaw Rate')
axarr[1, 2].plot(time, est_yaw_rate, 'r', label='Yaw Rate by Python')
axarr[1, 2].plot(datalog_ts, rt_est_dphi, 'y', label='Estimated RT Yaw Rate')
axarr[1, 2].set_xlabel("time [s]")
axarr[1, 2].set_ylabel("yaw rate [rad/s]")
# Put legend
axarr[1, 2].legend(loc='upper left')

###############################################################################
axarr[1, 0].set_title('Robot Speed')
axarr[1, 0].plot(time, odom_speed, 'b', label='Odometry Speed')
axarr[1, 0].plot(time, est_speed, 'r', label='Estimated Speed')
axarr[1, 0].plot(datalog_ts, rt_est_v, 'y', label='Estimated RT Speed')
#axarr[1, 0].set_ylim(-0.5, 1.2)
axarr[1, 0].set_xlabel("time [s]")
axarr[1, 0].set_ylabel("speed [m/s]")
# Put legend
axarr[1, 0].legend(loc='upper left')

axarr[1, 1].set_title('Acceleration')
axarr[1, 1].plot(datalog_ts, accel_x, 'g', label='IMU Accel datalog')
axarr[1, 1].plot(time, est_accel, 'r', label='Accel by Python')
axarr[1, 1].plot(datalog_ts, rt_est_a, 'y', label='Estimated RT Accel')
#axarr[1, 1].set_ylim(-10, 10)
axarr[1, 1].set_xlabel("time [s]")
axarr[1, 1].set_ylabel("speed [m/s]")
# Put legend
axarr[1, 1].legend(loc='upper left')