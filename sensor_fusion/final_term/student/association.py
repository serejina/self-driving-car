# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
             
        ############
        # Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        
        # the following only works for at most one track and one measurement
        self.association_matrix = np.matrix([]) # reset matrix
        self.unassigned_tracks = [] # reset lists
        self.unassigned_meas = []

        N = len(track_list) # N tracks
        M = len(meas_list) # M measurements
        tracks_range = range(N)
        meas_range = range(M)

        # update list of unassigned measurements and unassigned tracks
        if len(meas_list) > 0:
            self.unassigned_meas = list(meas_range)
        if len(track_list) > 0:
            self.unassigned_tracks = list(tracks_range)

        if len(meas_list) > 0 and len(track_list) > 0:
            # initialize association matrix
            self.association_matrix = np.inf*np.ones((N,M))

        # loop over all tracks and all measurements to set up association matrix
        for i in tracks_range:
            track = track_list[i]
            for j in meas_range:
                meas = meas_list[j]
                dist = self.MHD(track, meas, KF)
                if self.gating(dist, meas.sensor):
                    self.association_matrix[i,j] = dist

    def get_closest_track_and_meas(self):
        ############
        # Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
        ############

        # the following only works for at most one track and one measurement
        update_track = 0
        update_meas = 0
        A = self.association_matrix
        # NAN if no more associations can be found (i.e. minimum entry in association matrix is infinity)
        if np.min(A) == np.inf:
            return np.nan, np.nan

        # find indices of closest track and measurement for next update
        min_ij = np.unravel_index(np.argmin(A, axis=None), A.shape)
        track_idx = min_ij[0]
        meas_idx = min_ij[1]
        # delete row and column in association matrix for closest track and measurement
        A = np.delete(A, track_idx, 0)
        A = np.delete(A, meas_idx, 1)
        self.association_matrix = A
        
        # remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        update_track = self.unassigned_tracks[track_idx]
        update_meas = self.unassigned_meas[meas_idx]

        self.unassigned_tracks.remove(update_track) 
        self.unassigned_meas.remove(update_meas)

        return update_track, update_meas     

    def gating(self, MHD, sensor): 
        ############
        # Step 3: return True if measurement lies inside gate, otherwise False
        ############
        limit = chi2.ppf(params.gating_threshold, df=sensor.dim_meas)
        if MHD < limit:
            return True
        else:
            return False

    def MHD(self, track, meas, KF):
        ############
        # Step 3: calculate and return Mahalanobis distance
        ############
        H = meas.sensor.get_H(track.x)
        gamma = KF.gamma(track, meas)
        S = KF.S(track, meas, H)
        MHD = gamma.transpose() * np.linalg.inv(S) * gamma
        return MHD
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)