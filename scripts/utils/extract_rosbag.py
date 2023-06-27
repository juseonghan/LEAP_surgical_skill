import rosbag
import bagpy
import pandas as pd
import cv2 as cv
import os 
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import numpy as np

def main():

    # first read in {}_system.bag into csvs
    bag_path = Path('/home/juseonghan/leap/data/neobladder/user_3_a_c_system.bag')
    bag = bagpy.bagreader(str(bag_path))

    left_pose = pd.read_csv(bag.message_by_topic('/dvrk/MTML/position_cartesian_current'))
    left_twist = pd.read_csv(bag.message_by_topic('/dvrk/MTML/twist_body_current'))
    left_jac = pd.read_csv(bag.message_by_topic('/dvrk/MTML/jacobian_body'))

    right_pose = pd.read_csv(bag.message_by_topic('/dvrk/MTMR/position_cartesian_current'))
    right_twist = pd.read_csv(bag.message_by_topic('/dvrk/MTMR/twist_body_current'))
    right_jac = pd.read_csv(bag.message_by_topic('/dvrk/MTMR/jacobian_body'))

    psm1_pose = pd.read_csv(bag.message_by_topic('/dvrk/PSM1/position_cartesian_current'))
    psm1_twist = pd.read_csv(bag.message_by_topic('/dvrk/PSM1/twist_body_current'))
    psm1_jac = pd.read_csv(bag.message_by_topic('/dvrk/PSM1/jacobian_body'))
    psm1_jaw = pd.read_csv(bag.message_by_topic('/dvrk/PSM1/state_jaw_current'))

    psm2_pose = pd.read_csv(bag.message_by_topic('/dvrk/PSM2/position_cartesian_current'))
    psm2_twist = pd.read_csv(bag.message_by_topic('/dvrk/PSM2/twist_body_current'))
    psm2_jac = pd.read_csv(bag.message_by_topic('/dvrk/PSM2/jacobian_body'))
    psm2_jaw = pd.read_csv(bag.message_by_topic('/dvrk/PSM2/state_jaw_current'))

    sys_bicoag = pd.read_csv(bag.message_by_topic('/dvrk/footpedals/bicoag'))
    sys_camera = pd.read_csv(bag.message_by_topic('/dvrk/footpedals/camera'))
    sys_clutch = pd.read_csv(bag.message_by_topic('/dvrk/footpedals/clutch'))
    sys_operator = pd.read_csv(bag.message_by_topic('/dvrk/footpedals/operatorpresent'))

    # read all csvs 
    left_poses = read_PoseStamped(left_pose)
    left_twists = read_twistStamped(left_twist)
    left_jacs = read_jacobian(left_jac)

    right_poses = read_PoseStamped(right_pose)
    right_twists = read_twistStamped(right_twist)
    right_jacs = read_jacobian(right_jac)

    psm1_poses = read_PoseStamped(psm1_pose)
    psm1_twists = read_twistStamped(psm1_twist)
    psm1_jacs = read_jacobian(psm1_jac)
    psm1_jaws = read_jaw(psm1_jaw)

    psm2_poses = read_PoseStamped(psm2_pose)
    psm2_twists = read_twistStamped(psm2_twist)
    psm2_jacs = read_jacobian(psm2_jac)
    psm2_jaws = read_jaw(psm2_jaw)

    

    ## read in {}_video-{}.bag
    bag_path = '/home/juseonghan/leap/data/neobladder/user_3_a_c_video-002.bag'

def read_PoseStamped(pd_frame):

    result = []

    # read in all fields
    pos_x = pd_frame['pose.position.x']
    pos_y = pd_frame['pose.position.y']
    pos_z = pd_frame['pose.position.z']
    ori_x = pd_frame['pose.orientation.x']
    ori_y = pd_frame['pose.orientation.y']
    ori_z = pd_frame['pose.orientation.z']
    ori_w = pd_frame['pose.orientation.w']

    for i in range(len(pos_x)):
        # convert quat to rotation matrix
        r = R.from_quat(ori_x[i], ori_y[i], ori_z[i], ori_w[i])
        so3_mat = r.as_matrix().flatten()
        temp = [pos_x[i], pos_y[i], pos_z[i]] + so3_mat
        result.append(np.array(temp))
    
    return result
    
def read_twistStamped(pd_frame):
    result = []

    vel_x = pd_frame['twist.linear.x']
    vel_y = pd_frame['twist.linear.y']
    vel_z = pd_frame['twist.linear.z']
    vel_x_ori = pd_frame['twist.angular.x']
    vel_y_ori = pd_frame['twist.angular.y']
    vel_z_ori = pd_frame['twist.angular.z']

    for i in range(len(vel_x)):
        temp = np.array([vel_x[i], vel_y[i], vel_z[i], vel_x_ori[i], vel_y_ori[i], vel_z_ori[i]])
        result.append(temp)

    return result

def read_jacobian(pd_frame):

    arrs = pd_frame['data']
    return arrs

def read_jaw(pd_frame):
    jaw_vels = pd_frame['velocity']
    return jaw_vels


if __name__ == "__main__":
    main()
