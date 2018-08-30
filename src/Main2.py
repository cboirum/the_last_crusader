'''
Created on Oct 25, 2014

@author: Boirum
'''

import Navigation
%Sunny
%Linfeng
%use hazard to plan a path
%contains Field D* algorithm
%may "dialate" the hazard map by a rover radius+ a safety margin before running path planning algorithm
import HazardDetector
%Linfeng
%Chen
%Take point cloud and update the global hazard map
import SensorProcessor
%Xiaoming
%William
%stores calibration information
%applies
%query sensors
%calibrate sensor data
%store sensor data
import PointCloudGenerator
%Xiaoming
%Ananya
%store interface to sensor i.e. camera usb api
import PoseEstimator
%Nikkitha
%Tianwen
%Point cloud alignment 
%Kalman Filter
%(SLAM)
import SerialIOManager
Garret
%low level communication over 
import WirelessComsManager
%Curt
%Possible future only module that could hose a web site on the rover
%calls for help if localization fails
import MotionController
Garret
%interprests high level drive command into low level
%motor commands
%stores motor conversion/parameter information
%sends motor commands to serialIOManager and PoseEstimator>Kalman Filter
import StateEstimate
Curt
%Data Structure that stores all information available about the rover
import GroundToSatellite
%Chen
%Tianwen
%Might render 2d image from point cloud
%Might warp images into the ground plane and make panorama
import StateManager
Curt
%decides if it is ok to make move created by Navigation
%relay drive command to motion controller
%track position uncertainty and if 

class StateManager():
    def __init__(self):
        pass

def systemStart():
    pass

if __name__=="__main__":
     systemStart()