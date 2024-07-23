#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @brief    pick up a cube of a random position A and place t to random position B
# @author   Longfei Zhao (longfei.zhao.work@gmail.com)   

import sys
sys.dont_write_bytecode = True
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__),"../../common/imp")))
import random
import threading
import time
import numpy as np

import rospy
from dsr_msgs.msg import RobotStop, RobotState
from dsr_msgs.srv import Robotiq2FMove
import DR_init
DR_init.__dsr__id = 'dsr01'
DR_init.__dsr__model = 'm1013'
from DSR_ROBOT import _topic_name_prefix, _srv_name_prefix, posj, movej, posx, movel, DR_MV_MOD_ABS, DR_BASE

import transformations as tf

MIN_DIST = 500*1.414 # minimum distances between two tables

class PickCube():
    def __init__(self):
        rospy.init_node('poc_pick_cube')
        self.pub_stop = rospy.Publisher(_topic_name_prefix + '/stop', RobotStop, queue_size=10)  
        self.srv_gripper = rospy.ServiceProxy(_srv_name_prefix + '/gripper/robotiq_2f_move', Robotiq2FMove)
        rospy.on_shutdown(self.shutdown)

        output_thread = threading.Thread(target=self.state_update)
        output_thread.daemon = True
        output_thread.start()
        self.flag_output = False

        self.cube_attached = False


    def shutdown(self):
        print('poc_pick_cube is terminated!')
        self.pub_stop.publish(stop_mode=1)
        return 0
    

    def state_update(self):
        rospy.Subscriber(_topic_name_prefix + '/state', RobotState, self.state_cb)
        self.print_timer = time.time()
        rospy.spin()


    def state_cb(self, msg):
        if self.flag_output and time.time() - self.print_timer > 5:
            self.print_timer = time.time()

            if self.cube_attached:
                self.cube_pos = msg.current_posx
            hmat_cube_pose = self.posx2hmatrix(self.cube_pos)

            print('\n===========================================================================================')
            print(f'Start potion of the cube is             {self.pick_pos[:3]}')
            print(f'Expected desitation in drop zone is     {self.place_pos[:3]}')
            print(f'Current cube position is                {list(self.cube_pos[:3])}')
            print('')
            print(f'Start up-vector of cube is              {self.hmat_cube_start[:3, 0]}')
            print(f'Current cube up-vector is               {hmat_cube_pose[:3,0]}')
            print('===========================================================================================\n')


    def set_tables(self):
        tableAxy = [self.rand_xy(), self.rand_xy()]
        tableBxy = [self.rand_xy(), self.rand_xy()]
        while not self.valid_distance(tableAxy, tableBxy) and not rospy.is_shutdown():
            tableBxy = [self.rand_xy(), self.rand_xy()]

        tableAw = self.rand_w(tableAxy)
        tableBw = self.rand_w(tableBxy)

        # frame attached to the center of tables
        self.tableA = tableAxy + [300, tableAw, 0, 0]
        self.tableB = tableBxy + [300, tableBw, 0, 0]
        print(f'Spawned random location of tableA: {self.tableA}')
        print(f'Spawned random location of tableB: {self.tableB}')


    def gen_traj_pos(self):
        appr_dist = 100
        cube_size = 40
        table_size = 500

        cube_offset_A = [0, 0, cube_size/2, 0, 90, 0] # cube in center of table A
        cube_offset_B = [cube_size/2, table_size-cube_size/2, cube_size/2, 0, 90, 0]

        self.pick_pos = self.hmatrix2posx(tf.concatenate_matrices(self.posx2hmatrix(self.tableA), self.posx2hmatrix(cube_offset_A)))

        self.place_pos = self.hmatrix2posx(tf.concatenate_matrices(self.posx2hmatrix(self.tableB), self.posx2hmatrix(cube_offset_B), tf.rotation_matrix(np.pi, (0, 0, 1)))) # invert up-vector for cube

        self.pick_pos_appr, self.place_pos_appr = self.pick_pos[:], self.place_pos[:]
        self.cube_pos = self.pick_pos[:]
        self.hmat_cube_start = self.posx2hmatrix(self.cube_pos)
        
        self.pick_pos_appr[2] += appr_dist
        self.place_pos_appr[2] += appr_dist


    def rand_xy(self):
        return random.choice([-1,1])*random.randint(500, 750)


    def rand_w(self, Pxy):
        # since cube is symetric, any planar rotation can be wrapped in to [-45, 45] degree with an arbitrary offset
        return (random.uniform(-45.0, 45.0) + np.degrees(np.arctan2(Pxy[1], Pxy[0]))) % 360.0


    def posx2hmatrix(self, pos):
        ez, ey, ex = np.radians(pos[3:])
        hmatrix = tf.euler_matrix(ez, ey, ex, 'rzyx')
        hmatrix[0,3] = pos[0]
        hmatrix[1,3] = pos[1]
        hmatrix[2,3] = pos[2]
        return hmatrix
    

    def hmatrix2posx(self, hmatrix):
        position = [hmatrix[0,3], hmatrix[1,3], hmatrix[2,3]]
        w, p, r = tf.euler_from_matrix(hmatrix, 'rzyx')
        return posx(position + list(np.degrees([w,p,r])))


    def valid_distance(self, posA, posB):
        if (posA[0]-posB[0])**2 + (posA[1]-posB[1])**2 > MIN_DIST**2 and (posA[0]*posB[0]>0 or posA[1]*posB[1]>0):
            return True
        else:
            return False


    def procedure(self):
        # initial test to drive robot
        j_zero = posj(0, 0, 0, 0, 0, 0)
        j_home = posj(0, 0, 90, 0, 90, 0)

        vel_slow, vel_fast = [100, 100], [500, 500]
        acc_slow, acc_fast = [1000, 1000], [5000, 5000]
        
        movej(j_home, vel=180, acc=360)
        self.srv_gripper(0.0)

        while not rospy.is_shutdown():
            rospy.loginfo('Set random location and key points')
            self.set_tables() # set random location
            self.gen_traj_pos()
            self.flag_output = True

            rospy.loginfo('Apporach and reach the pick location')
            movel(self.pick_pos_appr, vel_fast, acc_fast, time=10, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            movel(self.pick_pos, vel_slow, acc_slow, time=2, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            rospy.loginfo('Grasp the object and rais the arm')
            self.srv_gripper(1.0)
            movel(self.pick_pos_appr, vel_slow, acc_slow, time=2, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            self.cube_attached = True

            rospy.loginfo('Apporach and reach to place location')
            movel(self.place_pos_appr, vel_fast, acc_fast, time=10, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            movel(self.place_pos, vel_slow, acc_slow, time=2, ref=DR_BASE, mod=DR_MV_MOD_ABS)

            rospy.loginfo('Release the object and rais the arm')
            movel(self.place_pos_appr, vel_slow, acc_slow, time=2, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            self.srv_gripper(0.0)
            self.cube_attached = False

            rospy.loginfo('Return home position for next cycle')
            movej(j_home, vel=180, acc=360)
            self.flag_output = False

            input('Press Enter to continue another round ...')


if __name__ == '__main__':
    test = PickCube()
    test.procedure()

    print('Exit main!')
