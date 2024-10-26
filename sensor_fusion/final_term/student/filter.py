# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # Step 1: implement and return system matrix F
        ############
        dt = params.dt
        return np.matrix([[1, 0 , 0, dt, 0, 0],
                          [0, 1 , 0, 0, dt, 0],
                          [0, 0 , 1, 0, 0, dt],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]])

    def Q(self):
        ############
        #1: implement and return process noise covariance Q
        ############
        q = params.q
        dt = params.dt
        q_1 = q * ((dt**3) / 3)
        q_2 = q * ((dt**2) / 2)
        q_3 = q * dt
        return np.matrix([[q_1, 0 , 0, q_2, 0, 0],
                          [0, q_1 , 0, 0, q_2, 0],
                          [0, 0 , q_1, 0, 0, q_2],
                          [q_2, 0, 0, q_3, 0, 0],
                          [0, q_2, 0, 0, q_3, 0],
                          [0, 0, q_2, 0, 0, q_3]])

    def predict(self, track):
        ############
        #Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        F = self.F()
        Q =  self.Q()
        x = F * track.x # state prediction
        P = F * track.P * F.transpose() + Q  # covariance prediction
        # save x and P in track
        track.set_x(x)
        track.set_P(P)

    def update(self, track, meas):
        ############
        #Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x)
        S = self.S(track, meas, H)
        K = track.P * H.transpose() * np.linalg.inv(S)  # Kalman gain
        x = track.x + K * self.gamma(track, meas)
        I = np.identity(params.dim_state)
        P = (I - K*H) * track.P # covariance update
        # save x and P in track
        track.set_x(x)
        track.set_P(P)
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        #Step 1: calculate and return residual gamma
        ############
        return meas.z - meas.sensor.get_hx(track.x)

    def S(self, track, meas, H):
        ############
        #Step 1: calculate and return covariance of residual S
        ############
        return H * track.P * H.transpose() + meas.R