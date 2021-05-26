
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
import simple_walk
import simple_trot
import creep
import idle_trot

import input_coordinates

class InputTask:
    def __init__(self, trajectory_queue, message_queue, shared_position, standard_input, masterboard_started_event):
        self.trajectory_queue = trajectory_queue
        self.message_queue = message_queue
        self.standard_input = standard_input

        self.shared_position = shared_position

        self.masterboard_started_event = masterboard_started_event

        self._running = True

        self._mode = "Idle"

    def run(self):
        # Priority settings for "InputTask" process
        #print(f"Input task started with pid: {os.getpid()} and priority: {os.sched_getparam(os.getpid())}")

        pid = os.getpid()
        sched = os.SCHED_FIFO
        param = os.sched_param(50)

        os.sched_setscheduler(pid, sched, param)

        #print(f"Input task with pid: {os.getpid()} changed to priority: {os.sched_getparam(os.getpid())}\n")

        sys.stdin = os.fdopen(self.standard_input)

        print()
        print(f"Robot is currently in {self._mode} mode")

        #Make sure master board has recorded a position before we try to read it
        self.masterboard_started_event.wait()
        
        with self.shared_position.get_lock():
                initial_position = self.shared_position[:]
        
        while(self._running):
            try:
                input_string = input(">")

                joint_variables, input_message = parser.interpret_input(input_string, self._mode, initial_position)

            except EOFError:
                self.terminate()
                continue
            
            if input_message == "simple walk":
                if self._mode != "Walk":
                    print(f"Currently in {self._mode}, change mode.")
                    continue
                initial_position = self.walk(initial_position, joint_variables)

            elif input_message == "simple trot":
                initial_position = self.trot(initial_position, joint_variables, self._mode)

            elif input_message == "Creep":
                if self._mode != "Idle":
                    print(f"Currently in {self._mode}, change mode.")
                initial_position = self.creep(initial_position, joint_variables)

            elif input_message == "Send":
                if self._mode == "Walk":
                    zero_config = kinematics.walk_zero_config
                    elbows = kinematics.walk_elbow
                    joint_polarities = kinematics.walk_joint_polarities
                else:
                    #return None, "InputError"

                    zero_config = kinematics.idle_zero_config
                    elbows = kinematics.idle_elbow
                    joint_polarities = kinematics.idle_joint_polarities

                importlib.reload(input_coordinates)

                coordinates = np.array(input_coordinates.coordinates, dtype=float)
                velocity = np.array(input_coordinates.velocity, dtype=float)
                acceleration = np.array(input_coordinates.acceleration, dtype=float)

                points = kinematics.simple_walk_controller(coordinates, initial_position, zero_config, joint_polarities, elbows)

                if None in points:
                    continue

                initial_position = self.que_trajectory(initial_position, points, velocity, acceleration)


            elif input_message != "":
                initial_position = self.handle_input(input_message, initial_position)

            elif input_message == "InputError":
                continue

            else:
                initial_position = self.que_trajectory(initial_position, joint_variables)

        sys.stdin.close()

        #print(f"Input task with pid: {os.getpid()} terminated")
    
    def terminate(self):
        self._running = False

    def handle_input(self, input_message, initial_position):

        if input_message == "Stand":
            initial_position = self.stand(self._mode, initial_position)

        elif input_message == "Home":
            initial_position = self.home(self._mode, initial_position)

        elif input_message == "Current":
            print(initial_position)

        elif input_message == "Walk" or input_message == "Idle":
            initial_position = self.change_mode(input_message, initial_position)

        elif input_message == "Exit":
            self.message_queue.put(input_message)  
        else:
            self.message_queue.put(input_message)

        return initial_position


    def creep(self, initial_position, num_cycles):
        zero_config = kinematics.idle_zero_config
        elbows = kinematics.idle_elbow
        joint_polarities = kinematics.idle_joint_polarities


        home = kinematics.simple_walk_controller(creep.home, initial_position, zero_config, joint_polarities, elbows)
        initial_position = self.que_trajectory(initial_position, home)

        for i in range(num_cycles):
            #print("Walk cycle")

            HL_lowering = kinematics.simple_walk_controller(creep.HL_lowering, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, HL_lowering, k_v = creep.velocity, k_a = creep.acceleration)

            HL_lifting = kinematics.simple_walk_controller(creep.HL_lifting, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, HL_lifting, k_v = creep.velocity, k_a = creep.acceleration)

            HL_home = kinematics.simple_walk_controller(creep.HL_home, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, HL_home, k_v = creep.velocity, k_a = creep.acceleration)

            HR_lowering = kinematics.simple_walk_controller(creep.HR_lowering, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, HR_lowering, k_v = creep.velocity, k_a = creep.acceleration)

            HR_lifting = kinematics.simple_walk_controller(creep.HR_lifting, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, HR_lifting, k_v = creep.velocity, k_a = creep.acceleration)

            HR_home = kinematics.simple_walk_controller(creep.HR_home, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, HR_home, k_v = creep.velocity, k_a = creep.acceleration)

            FR_lowering = kinematics.simple_walk_controller(creep.FR_lowering, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, FR_lowering, k_v = creep.velocity, k_a = creep.acceleration)

            FR_lifting = kinematics.simple_walk_controller(creep.FR_lifting, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, FR_lifting, k_v = creep.velocity, k_a = creep.acceleration)

            FR_home = kinematics.simple_walk_controller(creep.FR_home, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, FR_home, k_v = creep.velocity, k_a = creep.acceleration)

            FL_lowering = kinematics.simple_walk_controller(creep.FL_lowering, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, FL_lowering, k_v = creep.velocity, k_a = creep.acceleration)

            FL_lifting = kinematics.simple_walk_controller(creep.FL_lifting, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, FL_lifting, k_v = creep.velocity, k_a = creep.acceleration)

            FL_home = kinematics.simple_walk_controller(creep.FL_home, initial_position, zero_config, joint_polarities, elbows)            
            initial_position = self.que_trajectory(initial_position, FL_home, k_v = creep.velocity, k_a = creep.acceleration)

            home = kinematics.simple_walk_controller(creep.home, initial_position, zero_config, joint_polarities, elbows)
            initial_position = self.que_trajectory(initial_position, home, k_v = creep.velocity, k_a = creep.acceleration)

        home = kinematics.simple_walk_controller(simple_walk.home, initial_position, zero_config, joint_polarities, elbows)
        initial_position = self.que_trajectory(initial_position, home)

        return initial_position

    def walk(self, initial_position, num_cycles):
        importlib.reload(simple_walk)

        home = kinematics.simple_walk_controller(simple_walk.home, initial_position)
        initial_position = self.que_trajectory(initial_position, home)
        acceleration = simple_walk.acceleration


        for i in range(num_cycles):
            #print("Walk cycle")

            down = kinematics.simple_walk_controller(simple_walk.left_leg_down, initial_position)
            back = kinematics.simple_walk_controller(simple_walk.left_leg_back, initial_position)

            forward_to_back = np.vstack((down,back))

            initial_position = self.que_trajectory(initial_position, forward_to_back, k_v = simple_walk.forward_to_back_velocity, k_a = acceleration)

            up = kinematics.simple_walk_controller(simple_walk.left_leg_up, initial_position)
            forward = kinematics.simple_walk_controller(simple_walk.left_leg_forward, initial_position)

            back_to_forward = np.vstack((up, forward))

            initial_position = self.que_trajectory(initial_position, back_to_forward, k_v = simple_walk.back_to_forward_velocity, k_a = acceleration)

            #print(down, back, up, forward)

        home = kinematics.simple_walk_controller(simple_walk.home, initial_position)
        initial_position = self.que_trajectory(initial_position, home)

        return initial_position

    def trot(self, initial_position, num_cycles, mode):
        # Trot settings for Idle mode

        if mode == "Idle":
            importlib.reload(idle_trot) #Enables live update of varibles when testing trot settings

            zero_config = kinematics.idle_zero_config
            elbows = kinematics.idle_elbow
            joint_polarities = kinematics.idle_joint_polarities

            front_left_up_coordinates = idle_trot.HR_FL
            front_right_up_coordinates = idle_trot.HL_FR
            home_coordinates = idle_trot.home

            FL_up_velocity = idle_trot.HR_FL_velocity
            FR_up_velocity = idle_trot.HL_FR_velocity

            k_a = idle_trot.acceleration

        # Trot settings for Walk mode
        elif mode == "Walk":
            importlib.reload(simple_trot) #Enables live update of varibles when testing trot settings

            zero_config = kinematics.walk_zero_config
            elbows = kinematics.walk_elbow
            joint_polarities = kinematics.walk_joint_polarities

            front_left_up_coordinates = simple_trot.front_left_up
            front_right_up_coordinates = simple_trot.front_right_up
            home_coordinates = simple_trot.home

            FL_up_velocity = simple_trot.velocity
            FR_up_velocity = simple_trot.velocity

            k_a = simple_trot.acceleration

        # Calculate angles and que trajectory
        home = kinematics.simple_walk_controller(home_coordinates, initial_position, zero_config, joint_polarities, elbows)
        initial_position = self.que_trajectory(initial_position, home)

        for i in range(num_cycles):

            front_left_up = kinematics.simple_walk_controller(front_left_up_coordinates, initial_position, zero_config, joint_polarities, elbows)

            initial_position = self.que_trajectory(initial_position, front_left_up, k_v = FL_up_velocity, k_a = k_a)

            front_right_up = kinematics.simple_walk_controller(front_right_up_coordinates, initial_position, zero_config, joint_polarities, elbows)

            initial_position = self.que_trajectory(initial_position, front_right_up, k_v = FR_up_velocity, k_a = k_a)

        home = kinematics.simple_walk_controller(home_coordinates, initial_position, zero_config, joint_polarities, elbows)
        initial_position = self.que_trajectory(initial_position, home)

        return initial_position

    def stand(self, mode, initial_position):
        if mode == "Walk":
            points = config.walk_stand

        elif mode == "Idle":
            points = config.idle_stand

        initial_position = self.que_trajectory(initial_position, points = points, k_v = config.up_speed, k_a = config.up_acceleration)

        return initial_position

    def home(self, mode, initial_position):
        if mode == "Walk":
            points = config.walk_home

        elif mode == "Idle":
            points = config.idle_home

        initial_position = self.que_trajectory(initial_position, points = points, k_v = config.down_speed, k_a = config.down_acceleration)

        return initial_position

    def change_mode(self, new_mode, initial_position):
        if self._mode == new_mode:
            return initial_position

        initial_position = self.home(self._mode, initial_position)
        self._mode = new_mode
        initial_position = self.home(self._mode, initial_position)

        print()
        print(f"Robot is now in {self._mode} mode")

        return initial_position

    def que_trajectory(self, initial_position, points, k_v=config.default_speed, k_a=config.default_acceleration):
        trajectory = trj.Trajectory(points=[initial_position, points], k_v=k_v, k_a=k_a) 
        new_initial_position = trajectory.q_f


        if trajectory.is_valid():
            self.trajectory_queue.put(trajectory)

        return new_initial_position

class MasterBoardTask:
    def __init__(self, trajectory_queue, message_queue, shared_position, masterboard_started_event):
        #Local variables
        self._running = True
        self._terminating = False

        self.shared_position = shared_position

        self.trajectory_queue = trajectory_queue
        self.message_queue = message_queue

        self.masterboard_started_event = masterboard_started_event

        self.debug_plot = False

        self.balance_mode = False

        #self.plot = plotter.Plotter("Test Plot")

        #for i in range(config.N_SLAVES_CONTROLLED * 2):
        #    self.plot.create_plot(f"Joint {i}, Current Reference", "Current [A]")
        #    self.plot.create_plot(f"Joint {i}, Position Set Point", "Position [rad]")
        #    self.plot.create_plot(f"Joint {i}, Position Reading", "Position [rad]")
        #    self.plot.create_plot(f"Joint {i}, Position Error", "position [rad]")

        self.master_board = masterboard.MasterBoard()

        if self.debug_plot:
            self.plotter = pltr.Plotter("First walk")

            for i in range(config.N_SLAVES_CONTROLLED * 2):
                self.plotter.create_plot(f"Joint {i}, Position Set Point", "Position [rad]")
                #self.plotter.create_plot(f"Joint {i}, Position Reading", "Position [rad]")
                #self.plotter.create_plot(f"Joint {i}, Position Error", "position [rad]")

                self.plotter.create_plot(f"Joint {i}, Current Reference", "Current [A]")

                #self.plotter.create_plot(f"Joint {i}, Velocity Set Point", "Velocity [rad/s]")
                #self.plotter.create_plot(f"Joint {i}, Velocity Reading", "Velocity [rad/s]")
                #self.plotter.create_plot(f"Joint {i}, Velocity Error", "Velocity [rad/s]")
        

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
        
        with self.shared_position.get_lock():
            self.shared_position[:] = q

        self.masterboard_started_event.set()

        trajectory = None


        # Timer initiation
        cycle_time = 0
        program_start = time.perf_counter()
        timestep_start = time.perf_counter()
        timestep_end = time.perf_counter()
        cycle_start = time.perf_counter()

        self.prev_t = 0

        self.prev_att_err = 0

        self.acc_error = np.zeros(2)

        self.kp = 200 #200
        self.kd = 3.5   #2
        self.ki = 6000 #3000
        
        # Communication with MasterBoard
        while(self._running and not timeout):
            balance_mode = self.balance_mode

            #Shared position. This allows InputTask to read the current position from a shared array
            current_position = self.master_board.get_state()[0]
            with self.shared_position.get_lock():
                self.shared_position[:] = current_position

            if balance_mode:
                if not self.message_queue.empty():
                    input_message = self.message_queue.get(block=False)
                    self.handle_input(input_message, q)


                attitude = self.master_board.get_attitude()

                if self.prev_t == 0:
                    self.prev_t = current_time

                q, q_prime = self.balance(attitude, current_position, current_time)

                timeout = self.master_board.set_reference(q, q_prime)

            elif trajectory == None:

                

                if not self.message_queue.empty():
                    input_message = self.message_queue.get(block=False)
                    self.handle_input(input_message, q)

                # No trajectory, try to get new from queue.
                elif not self.trajectory_queue.empty():
                    trajectory = self.trajectory_queue.get(block=False) # Check queue
                    initial_trajectory_time = time.perf_counter()

                else:
                    if self._terminating:
                        self._running = False

                
            else:
                
                # Follow current trajectory
                
                time_before_initial_condition = time.perf_counter()

                current_trajectory_time = time.perf_counter() - initial_trajectory_time

                q, q_prime = trajectory.step(current_trajectory_time)

                ic_time_array[cpt] =  time.perf_counter() - time_before_initial_condition

                timeout = self.master_board.set_reference(q, q_prime)
                
                '''
                for i in range(len(q)):
                    if i < 2:
                        self.plotter.add_data(f"Joint {i}, Position Set Point", q[i])
                        self.plotter.add_data(f"Joint {i}, Velocity Set Point", q_prime[i])

                        #pos_reading, vel_reading = self.master_board.get_state()

                        #self.plotter.add_data(f"Joint {i}, Position Reading", pos_reading[i])
                        #self.plotter.add_data(f"Joint {i}, Velocity Reading", vel_reading[i])

                        #self.plotter.add_data(f"Joint {i}, Position Error", pos_reading[i] - q[i])
                        #self.plotter.add_data(f"Joint {i}, Velocity Error", vel_reading[i] - q_prime[i])
                
                '''
                if trajectory.is_done():
                    trajectory = None

            # Update current reference to each joint.
            if self.debug_plot:
                timeout = self.master_board.track_reference(self.plotter)
                self.plotter.add_time(current_time)
            else:
                timeout = self.master_board.track_reference(None)
            #self.plot.add_time(current_time)
            
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

        if self.debug_plot:
            for i in range(config.N_SLAVES_CONTROLLED):
                self.plotter.set_num_cols(2)

                self.plotter.create_axis([f"Joint {2*i}, Position Set Point"])

                self.plotter.create_axis([f"Joint {2*i}, Current Reference"])

                self.plotter.create_axis([f"Joint {2*i+1}, Position Set Point"])

                self.plotter.create_axis([f"Joint {2*i+1}, Current Reference"])

                #self.plotter.create_axis([ 
                #   f"Joint {2*i}, Velocity Set Point",
                #    f"Joint {2*i+1}, Velocity Set Point"
                #])
                
                self.plotter.plot(grid=True)
        

    def trajectory_queue_init(self):
        '''
        For å få lavere cycle time ved første trajectory
        '''
        trajectory = trj.Trajectory([0,0,0,0,0,0,0,0])
        self.trajectory_queue.put(trajectory)
        self.trajectory_queue.get()
        trajectory.set_initial_conditions(0, 0)

    def handle_input(self, input_message, q):
        if input_message == "Exit":
            print("Should exit")
            self._running = False

        elif input_message == "Position":
            self.print_current_position(q)

        elif input_message == "Balance":
            if self.balance_mode:
                self.balance_mode = False
            else:
                self.balance_mode = True
                self.prev_t = 0
                self.acc_error = np.zeros(2)

    def print_current_position(self, current_pos_ref):
        #Prints the current reference and position of all the motors
        #Meant to be called by the InputTask through the use of synchronized bools
        current_position = self.master_board.get_state()[0]
        current_actual_position = self.master_board.get_actual_state()[0]

        np.set_printoptions(suppress=True)

        # Display position in radians
        print("\n--------------------------POSITION [RAD]----------------------------------\n")
        print(f"Setpoint:\n{current_pos_ref.round(5).tolist()}\n")
        print(f"Current Postition (get_state):\n{current_position.round(5).tolist()}\n")
        print(f"Error = Current Position - Setpoint:\n{(current_position - current_pos_ref).round(6).tolist()}\n")
        print(f"\nActual Position (get_actual_state):\n{current_actual_position.round(5).tolist()}\n")

        #Display position in degress
        print("\n-------------------------POSITION [DEG]-----------------------------------\n")
        print(f"Setpoint ={np.degrees(current_pos_ref).round(5).tolist()}")
        print(f"Current Postition (get_state):\n{np.degrees(current_position).round(5).tolist()}")
        print(f"Error = Current Position - Setpoint:\n{np.degrees(current_position - current_pos_ref).round(5).tolist()}\n")
        print(f"\nActual Position (get_actual_state):\n{np.degrees(current_actual_position).round(5).tolist()}")
        print("\n--------------------------------------------------------------------------\n")

    def balance(self, attitude, current_position, t):
        att_ref = np.array([0.0,0.0])
        att_read = attitude[:2]

        att_err = att_ref - att_read

        if t != 0 and t != self.prev_t:
            att_err_prime = (att_err - self.prev_att_err) / (t - self.prev_t)
        else:
            att_err_prime = 0

        self.acc_error = self.acc_error + (self.ki * att_err * (t - self.prev_t))

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

        q = kinematics.simple_walk_controller(coordinates, current_position, zero_config=kinematics.idle_zero_config, joint_polarities=kinematics.idle_joint_polarities, elbows=kinematics.idle_elbow)

        if None in q:
            return current_position, np.zeros(8)

        return q, np.zeros(8)

    def terminate(self):
        print(f"Master board task with pid: {os.getpid()} terminated")
        self._running = False


masterboard_started_event = multiprocessing.Event()

shared_position = multiprocessing.Array(ctypes.c_double, 8)

standard_input = sys.stdin.fileno()

trajectory_queue = multiprocessing.Queue()
message_queue = multiprocessing.Queue()

input_task = InputTask(trajectory_queue, message_queue, shared_position, standard_input, masterboard_started_event)
master_board_task = MasterBoardTask(trajectory_queue, message_queue, shared_position, masterboard_started_event)

input_thread = multiprocessing.Process(target = input_task.run, daemon = True)
input_thread.start()

master_board_task.run()

os._exit(0)