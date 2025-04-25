import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
import matplotlib.pyplot as plt
from util import RotMatrix3D, point_to_rad

class kinematics():
    
    def __init__(self):
        
        # note: leg IDs
        left_front = 0
        left_back  = 1
        right_front= 2
        right_back = 3
        
        self.right_legs = [right_front, right_back]
        
        # Rename links using standard hexapod leg segment names
        self.coxa_length = 0.03    # previously link_1
        self.femur_length = 0.1115  # previously link_2
        self.tibia_length = 0.155   # previously link_3
        self.coxa_angle = radians(90)  # previously phi
        
        # body dimensions
        self.body_length = 0.25205
        self.body_width = 0.105577
        self.body_height = 0.0
        
        # leg origins (left_front, left_back, right_back, right_front), i.e., the coordinate of coxa_joint
        self.leg_origins = np.matrix([[self.body_length/2, self.body_width/2, 0],
                                     [-self.body_length/2, self.body_width/2, 0],
                                     [-self.body_length/2, -self.body_width/2, 0],
                                     [self.body_length/2, -self.body_width/2, 0],
                                     [self.body_length/2, self.body_width/2, 0]])
        
    # this method adjust inputs to the IK calculator by adding rotation and 
    # offset of that rotation from the center of the robot
    def leg_IK(self, end_effector_pos, rot = [0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
        
        # check is the leg is from the right side 
        is_right = (legID in self.right_legs)
        
        # add offset of each leg from the axis of rotation
        transformed_pos = asarray((inv(RotMatrix3D(rot, is_radians)) * \
            ((array(end_effector_pos) + self.leg_origins[legID,:] - array(center_offset)).transpose())).transpose())
        
        # subtract the offset between the leg and the center of rotation 
        # so that the resultant coordinate is relative to the origin (coxa_joint) of the leg
        relative_pos = asarray(transformed_pos - self.leg_origins[legID,:] + array(center_offset)).flatten()

        # calculate the angles and coordinates of the leg relative to the origin of the leg
        return self.leg_IK_calc(relative_pos, is_right)


    # IK calculator
    def leg_IK_calc(self, end_effector_pos, is_right=False): 

        x, y, z = end_effector_pos[0], end_effector_pos[1], end_effector_pos[2]    # unpack coordinates
        
        # length of vector projected on the YZ plane. equiv. to proj_length = sqrt(y**2 + z**2)
        proj_length = norm([0, y, z])   
        
        # angle_1 : angle from the positive y-axis to the end-effector (0 <= angle_1 < 2pi)
        # angle_2 : angle between proj_length and leg's projection line on YZ plane
        # angle_3 : angle between coxa and length proj_length
        angle_1 = point_to_rad(y, z)                     
        angle_2 = asin(sin(self.coxa_angle) * self.coxa_length / proj_length) 
        angle_3 = pi - angle_2 - self.coxa_angle                   
        
        # angle of coxa about the x-axis 
        if is_right: 
            coxa_angle = angle_1 - angle_3
        else: 
            coxa_angle = angle_1 + angle_3
            if coxa_angle >= 2*pi: 
                coxa_angle -= 2*pi
        
        coxa_joint = array([0, 0, 0]) # ADDED
        femur_joint = array([0, self.coxa_length * cos(coxa_angle), self.coxa_length * sin(coxa_angle)])
        end_joint = array(end_effector_pos)
        femur_to_end_vector = end_joint - femur_joint  # vector from femur_joint to end_joint
        
        if is_right: 
            rotation_angle = coxa_angle - self.coxa_angle - pi/2
        else: 
            rotation_angle = coxa_angle + self.coxa_angle - pi/2
        
        # create rotation matrix to work on a new 2D plane (XZ_)
        rot_mtx = RotMatrix3D([-rotation_angle, 0, 0], is_radians=True)
        rotated_vector = rot_mtx * (np.reshape(femur_to_end_vector, [3, 1]))
        
        # coordinates in the rotated coordinate system
        x_rot, y_rot, z_rot = rotated_vector[0], rotated_vector[1], rotated_vector[2]
        
        femur_tibia_distance = norm([x_rot, z_rot])  # distance from femur to end effector
        
        # handling mathematically invalid input, i.e., point too far away to reach
        if femur_tibia_distance >= (self.femur_length + self.tibia_length): 
            femur_tibia_distance = (self.femur_length + self.tibia_length) * 0.99999
            print('target coordinate: [%f %f %f] too far away' % (x, y, z))
        
        # vec_angle : angle between +ve x-axis and femur_tibia_distance (0 <= vec_angle < 2pi)
        # femur_angle : angle between femur_tibia_distance and femur
        # knee_angle : angle between femur and tibia
        vec_angle = point_to_rad(x_rot, z_rot)  
        femur_angle = acos((self.femur_length**2 + femur_tibia_distance**2 - self.tibia_length**2) / 
                          (2 * self.femur_length * femur_tibia_distance)) 
        knee_angle = acos((self.femur_length**2 + self.tibia_length**2 - femur_tibia_distance**2) / 
                         (2 * self.femur_length * self.tibia_length))  
        
        # assuming femur_angle = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
        femur_angle_corrected = vec_angle - femur_angle    
        tibia_angle = pi - knee_angle
        
        # CALCULATE THE COORDINATES OF THE JOINTS FOR VISUALIZATION
        coxa_joint_pos = np.array([0, 0, 0])
        
        # calculate tibia joint position
        tibia_joint_rotated = np.reshape(
            np.array([self.femur_length * cos(femur_angle_corrected), 0, self.femur_length * sin(femur_angle_corrected)]), 
            [3, 1]
        )
        tibia_joint_pos = np.asarray(
            femur_joint + np.reshape(np.linalg.inv(rot_mtx) * tibia_joint_rotated, [1, 3])
        ).flatten()
        
        # calculate end effector position
        end_rotated = tibia_joint_rotated + np.reshape(
            np.array([self.tibia_length * cos(femur_angle_corrected + tibia_angle), 0, 
                      self.tibia_length * sin(femur_angle_corrected + tibia_angle)]), 
            [3, 1]
        )
        end_effector_pos = np.asarray(
            femur_joint + np.reshape(np.linalg.inv(rot_mtx) * end_rotated, [1, 3])
        ).flatten()
        
        # modify angles to match robot's configuration (i.e., adding offsets)
        corrected_angles = self.angle_corrector(
            angles=[coxa_angle, femur_angle_corrected, tibia_angle], 
            is_right=is_right
        )
        
        return [corrected_angles[0], corrected_angles[1], corrected_angles[2], 
                coxa_joint_pos, femur_joint, tibia_joint_pos, end_effector_pos]
    
    
    def base_pose(self, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        
        # offset due to non-centered axes of rotation
        offset = RotMatrix3D(rot, is_radians) * \
            (matrix(center_offset).transpose()) - matrix(center_offset).transpose()
        
        # rotate the base around the center of rotation (if there is no offset, then the center of 
        # rotation will be at the center of the robot)
        rotated_base = RotMatrix3D(rot, is_radians) * self.leg_origins.transpose() - offset
        return rotated_base.transpose()
       
    # get coordinates of leg joints relative to coxa_joint
    def leg_pose(self, end_effector_pos, rot, legID, is_radians, center_offset=[0,0,0]):
        
        # get the coordinates of each joints relative to the leg's origin
        pose_relative = self.leg_IK(end_effector_pos, rot, legID, is_radians, center_offset)[3:]
        
        # adjust the coordinates according to the robot's orientation (roll, pitch, yaw)
        pose_true = RotMatrix3D(rot, is_radians) * (array(pose_relative).transpose())
        return pose_true.transpose()
    
    # plot rectangular base where each corner represents the origin of leg
    def plot_base(self, ax, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        # get coordinates
        p = (self.base_pose(rot, is_radians, center_offset)).transpose()     
        # plot coordinates
        ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'r')
        return
       
    # plot leg 
    def plot_leg(self, ax, end_effector_pos, rot=[0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
        # get coordinates
        p = ((self.leg_pose(end_effector_pos, rot, legID, is_radians, center_offset) \
                + self.base_pose(rot, is_radians, center_offset)[legID]).transpose())
        # plot coordinates
        ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'b')
        return

    def plot_robot(self, end_effector_positions, rot=[0,0,0], leg_N=4, is_radians=True, limit=0.250, center_offset=[0,0,0]):
    
        ax = self.ax_view(limit)  # set the view
        self.plot_base(ax, rot, is_radians, center_offset)  # plot base

        # plot legs
        for leg in range(leg_N):
            self.plot_leg(ax, end_effector_positions[leg], rot, leg, is_radians, center_offset) 
        
        # show figure
        plt.show()
        return

        
    # TO-DO : modify this function depending on your robot's configuration
    # adjusting angle for specific configurations of motors, incl. orientation
    # this will vary for each robot (possibly for each leg as well)
    def angle_corrector(self, angles=[0,0,0], is_right=True):
        angles[1] -= 1.5*pi;  # add offset 
        
        if is_right:
            corrected_coxa_angle = angles[0] - pi
            corrected_femur_angle = angles[1] + 45*pi/180  # 45 degrees initial offset
        else: 
            if angles[0] > pi:  
                corrected_coxa_angle = angles[0] - 2*pi
            else: 
                corrected_coxa_angle = angles[0]
            
            corrected_femur_angle = -angles[1] - 45*pi/180
        
        corrected_tibia_angle = -angles[2] 
        return [corrected_coxa_angle, corrected_femur_angle, corrected_tibia_angle]
        
    # set view  
    @staticmethod
    def ax_view(limit):
        ax = plt.axes(projection="3d")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        return ax