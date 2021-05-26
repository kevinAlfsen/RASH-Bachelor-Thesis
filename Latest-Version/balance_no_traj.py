
#Standard modules
import time
import matplotlib.pyplot as plt
import os
import numpy as np
import multiprocessing
import sys
import queue
import ctypes

import importlib

#RASH modules
import masterboard
import config

import Trajectory_Via as trj


import parser
import kinematics
import plotter as pltr


import queue

class MasterBoardTask:
    def __init__(self):
        #Local variables
        self._running = True

        self.master_board = masterboard.MasterBoard()

        self.plotter = pltr.Plotter("balance")
            
        self.plotter.create_plot("Attitude Reading", "Rotation [rad]")

        self.prev_att_err = 0
        self.prev_t = 0

        self.kp = 100 #200
        self.kd = 0.25
        self.ki = 4000 #3000

        self.current_coordinates = np.array([
            [0,100],
            [0,100],
            [0,100],
            [0,100]
        ])

        self.acc_error = np.zeros(2)

    def run(self):
        # Priority settings for "MasterBoardTask"
        print(f"Master board task started with pid: {os.getpid()} and priority: {os.sched_getparam(os.getpid())}")

        pid = os.getpid() # Identify process ID
        sched = os.SCHED_FIFO # FIFO real time scheduler
        param = os.sched_param(90) # Process priority

        os.sched_setscheduler(pid, sched, param) # Update priority settings

        print(f"Master board task with pid: {os.getpid()} changed to priority: {os.sched_getparam(os.getpid())}\n")

        # Local variables
        dt = config.dt
        timeout = False
        current_time = 0.0
        cpt = 0

        # Troubleshooting arrays (Normally not used)
        plot_duration = 1000

        plot_array_size = int(plot_duration / dt)

        time_per_timestep_array = np.zeros(plot_array_size + 1)
        cycle_time_array = np.zeros(plot_array_size + 1)


        # Trajectory and Queue initiation
        #self.trajectory_queue_init()

        q, _ = self.master_board.get_state()


        # Timer initiation
        cycle_time = 0
        program_start = time.perf_counter()
        timestep_start = time.perf_counter()
        timestep_end = time.perf_counter()
        cycle_start = time.perf_counter()

        self.prev_t = current_time
        

        self.acc_error = np.zeros(2)

        # Communication with MasterBoard
        while(self._running and not timeout):
            attitude = self.master_board.get_attitude()

            current_position = self.master_board.get_state()[0]

            q = self.balance(attitude, current_position, current_time)

            #print(np.degrees(q))

            timeout = self.master_board.set_reference(q, np.zeros(8))

            timeout = self.master_board.track_reference(None)

            self.plotter.add_data("Attitude Reading", attitude)

            self.plotter.add_time(current_time)
            
            # Cycle time is time spent calculating trajectory
            cycle_time = time.perf_counter() - cycle_start
            cycle_time_array[cpt] = cycle_time

            if cycle_time >= dt:
                cycle_time = dt

            # Sleep MasterBoard for 1ms. 
            time.sleep(dt - cycle_time) # ideally: dt - cycle_time = 1ms
            cycle_start = time.perf_counter() # Start time for next cycle 


            timestep_end = cycle_start
            time_per_timestep_array[cpt] = timestep_end - timestep_start
            timestep_start = cycle_start

            current_time = current_time + dt

            if cpt < plot_array_size:
                cpt = cpt + 1

        self.master_board.terminate()

        #Reset process priority
        sched = os.SCHED_OTHER
        param = os.sched_param(os.sched_get_priority_min(sched))
        os.sched_setscheduler(pid, sched, param)

        print(f"Master board task with pid: {os.getpid()} changed to priority: {os.sched_getparam(os.getpid())}")

        plt.plot(time_per_timestep_array[0:cpt - 1])
        plt.show()
        plt.plot(cycle_time_array[0:cpt - 1])
        plt.show()

    def terminate(self):
        print(f"Master board task with pid: {os.getpid()} terminated")
        self._running = False

    def balance(self, attitude, current_position, t):
        att_ref = np.array([0.0,0.0])
        att_read = attitude[:2]

        att_err = att_ref - att_read

        if t != 0:
            att_err_prime = (att_err - self.prev_att_err) / (t - self.prev_t)
        else:
            att_err_prime = 0

        self.acc_error = self.acc_error + (self.ki * att_err * (t - self.prev_t))

        #print(np.degrees(att_err))

        #print(self.ki * self.acc_error)

        delta_y = self.kp * att_err + self.kd * att_err_prime + self.acc_error

        self.prev_att_err = att_err
        self.prev_t = t

        delta_y = - delta_y

        coordinates = np.array([
            [0, 100 + delta_y[1]/2 + delta_y[0]/2],
            [0, 100 + delta_y[1]/2 - delta_y[0]/2],
            [0, 100 - delta_y[1]/2 + delta_y[0]/2],
            [0, 100 - delta_y[1]/2 - delta_y[0]/2]
        ])

       #print(coordinates)

        q = kinematics.simple_walk_controller(coordinates, current_position, zero_config=kinematics.idle_zero_config, joint_polarities=kinematics.idle_joint_polarities, elbows=kinematics.idle_elbow)

        if None in q:
            return current_position

        return q

master_board_task = MasterBoardTask()

master_board_task.run()

os._exit(0)





