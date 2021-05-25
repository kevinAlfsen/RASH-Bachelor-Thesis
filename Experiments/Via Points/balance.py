
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

        self.trajectory_queue = queue.Queue()

        self.master_board = masterboard.MasterBoard()

        self.plotter = pltr.Plotter("balance")

        for i in range(config.N_SLAVES_CONTROLLED * 2):
            self.plotter.create_plot(f"Joint {i}, Position Set Point", "Position [rad]")
        
        current_position = self.master_board.get_state()[0]

        home = np.array([
            [0,100],
            [0,100],
            [0,100],
            [0,100]
        ])

        new_point = kinematics.simple_walk_controller(home, current_position, zero_config=kinematics.idle_zero_config, joint_polarities=kinematics.idle_joint_polarities, elbows=kinematics.idle_elbow)
        self.trajectory_queue.put(trj.Trajectory((current_position, new_point)))

        self.initial_position = new_point

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
        ic_time_array = np.zeros(plot_array_size + 1)


        # Trajectory and Queue initiation
        #self.trajectory_queue_init()

        q, q_prime = self.master_board.get_state()

        trajectory = None


        # Timer initiation
        cycle_time = 0
        program_start = time.perf_counter()
        timestep_start = time.perf_counter()
        timestep_end = time.perf_counter()
        cycle_start = time.perf_counter()


        
        # Communication with MasterBoard
        while(self._running and not timeout):
            if trajectory == None:

                # No trajectory, try to get new from queue.
                if not self.trajectory_queue.empty():
                    trajectory = self.trajectory_queue.get(block=False) # Check queue
                    initial_trajectory_time = time.perf_counter()

                
            else:
                
                # Follow current trajectory
                
                time_before_initial_condition = time.perf_counter()

                current_trajectory_time = time.perf_counter() - initial_trajectory_time

                q, q_prime = trajectory.step(current_trajectory_time)

                ic_time_array[cpt] =  time.perf_counter() - time_before_initial_condition

                timeout = self.master_board.set_reference(q, q_prime)
                
                if trajectory.is_done():
                    trajectory = None

            # Update current reference to each joint.
            timeout = self.master_board.track_reference(None)

            for i in range(config.N_SLAVES_CONTROLLED * 2):
                self.plotter.add_data(f"Joint {i}, Position Set Point", q)

            self.plotter.add_time(current_time)

            attitude = self.master_board.get_attitude()

            if trajectory == None:
                self.initial_position = self.balance(attitude, self.initial_position)
            
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
        plt.plot(ic_time_array[0:cpt - 1])
        plt.show()

        for i in range(config.N_SLAVES_CONTROLLED):
            self.plotter.set_num_cols(2)

            self.plotter.create_axis([f"Joint {2*i}, Position Set Point"])

            self.plotter.plot(grid=True)

    def terminate(self):
        print(f"Master board task with pid: {os.getpid()} terminated")
        self._running = False

    def balance(self, attitude, initial_position):
        y_sp = 0
        l = 390

        home = np.array([
            [0,100],
            [0,100],
            [0,100],
            [0,100]
        ])

        y_pv = l * np.tan(attitude[1])

        y_err = y_pv - y_sp

        y_err2 = l * np.tan(attitude[0])

        coordinates = home + np.array([
            [0, y_err/2 + y_err2/2],
            [0, y_err/2 - y_err2/2],
            [0,-y_err/2 + y_err2/2],
            [0,-y_err/2 - y_err2/2]
        ])

        new_point = kinematics.simple_walk_controller(coordinates, initial_position, zero_config=kinematics.idle_zero_config, joint_polarities=kinematics.idle_joint_polarities, elbows=kinematics.idle_elbow)

        if None in new_point:
            return initial_position

        trajectory = trj.Trajectory(points=[initial_position, new_point])

        self.trajectory_queue.put(trj.Trajectory(points=[initial_position, new_point]))

        return new_point

master_board_task = MasterBoardTask()

master_board_task.run()

os._exit(0)

