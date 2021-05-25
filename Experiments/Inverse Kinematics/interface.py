
#Standard modules
import time
import matplotlib.pyplot as plt
import os
import numpy as np
import multiprocessing
import sys
import queue
import ctypes

#RASH modules
import masterboard
import config
import trajectory as trj
import parser
import kinematics
import plotter

class InputTask:
    def __init__(self, trajectory_queue, message_queue, shared_position, standard_input):
        self.trajectory_queue = trajectory_queue
        self.message_queue = message_queue
        self.standard_input = standard_input

        self.shared_position = shared_position

        self._running = True

        self._mode = "Idle"

    def run(self):
        # Priority settings for "InputTask" process
        print(f"Input task started with pid: {os.getpid()} and priority: {os.sched_getparam(os.getpid())}")

        pid = os.getpid()
        sched = os.SCHED_FIFO
        param = os.sched_param(50)

        os.sched_setscheduler(pid, sched, param)


        print(f"Input task with pid: {os.getpid()} changed to priority: {os.sched_getparam(os.getpid())}\n")

        sys.stdin = os.fdopen(self.standard_input)

        print()
        print(f"Robot is currently in {self._mode} mode")

        while(self._running):
            try:
                input_params, input_message = parser.parse_input(">")
            except EOFError:
                self.terminate()
                continue

            if input_message == "Angles":
                if self._mode == "Idle":
                    trajectory = trj.Trajectory(input_params)

                    if trajectory.is_valid():
                        self.trajectory_queue.put(trajectory)
                else:
                    print("Robot is not in idle mode. Send it to idle by typing >Idle")

            elif input_message == "Coordinates":
                if self._mode == "Walk":
                    with self.shared_position.get_lock(): 
                        current_position = self.shared_position[:]

                    joint_params = kinematics.simple_walk_controller(input_params, current_position)

                    if None in joint_params:
                        continue

                    #print(f"Going to : {joint_params} [RAD]")
                    #print(f"Joint angles : {np.degrees(joint_params)} [deg]")

                    trajectory = trj.Trajectory(joint_params)

                    if trajectory.is_valid():
                        self.trajectory_queue.put(trajectory)
                else:
                    print("Robot is not in walking mode. Type >Walk")
            else:
                self.handle_input(input_message)
                

        sys.stdin.close()

        #print(f"Input task with pid: {os.getpid()} terminated")
    def terminate(self):
        self._running = False

    def handle_input(self, input_message):
        if input_message == "Stand":
            trajectory = trj.Trajectory(q_f=config.stand_config, k_v=np.full(8, 20), k_a=np.full(8, 80))
            if trajectory.is_valid():
                self.trajectory_queue.put(trajectory)

        elif input_message == "Home":
            trajectory = trj.Trajectory(q_f=config.home_config, k_v=config.down_speed, k_a=config.down_acceleration)
            if trajectory.is_valid():
                self.trajectory_queue.put(trajectory)
        elif input_message == "Current":

            with self.shared_position.get_lock():
                print(self.shared_position[:])

        elif input_message == "Walk" or input_message == "Idle" or input_message == "Trot":
                self.change_mode(input_message)

        elif input_message == "Exit":
            while not self.trajectory_queue.empty():
                self.trajectory_queue.get()

            self.return_to_home(self._mode)
            self.message_queue.put(input_message) 

        else:
            self.message_queue.put(input_message)

    def return_to_home(self, mode):
        if mode == "Walk":
            home_trajectory  = trj.Trajectory(config.walk_mode_home)
        elif mode == "Idle":
            home_trajectory = trj.Trajectory(config.idle_mode_home)
        elif mode == "Trot":
            #At this moment, trot mode and idle mode have the same home config
            home_trajectory = trj.Trajectory(config.idle_mode_home) 

        self.trajectory_queue.put(home_trajectory)

    def change_mode(self, new_mode):
        if self._mode == new_mode:
            return

        self.return_to_home(self._mode)
        self._mode = new_mode
        self.return_to_home(self._mode)

        print()
        print(f"Robot is now in {self._mode} mode")




class MasterBoardTask:
    def __init__(self, trajectory_queue, message_queue, shared_position):
        #Local variables
        self._running = True
        self._terminating = False

        self.shared_position = shared_position

        self.trajectory_queue = trajectory_queue
        self.message_queue = message_queue

        self.plot = plotter.Plotter(f"Kinematics test | kp = {config.kp}, kd = {config.kd}, current saturation = {config.current_saturation}")

        for i in range(config.N_SLAVES_CONTROLLED * 2):
        #    self.plot.create_plot(f"Joint {i}, Current Reference", "Current [A]")
            self.plot.create_plot(f"Joint {i}, Position Reference", "Position [rad]")
            self.plot.create_plot(f"Joint {i}, Position Reading", "Position [rad]")
        #    self.plot.create_plot(f"Joint {i}, Position Error", "position [rad]")



        self.master_board = masterboard.MasterBoard(self.plot)

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
        self.trajectory_queue_init()

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

                time_before_initial_condition = time.perf_counter()

                if not self.message_queue.empty():
                    input_message = self.message_queue.get(block=False)
                    self.handle_input(input_message, q)

                # No trajectory, try to get new from queue.
                elif not self.trajectory_queue.empty():
                    trajectory = self.trajectory_queue.get(block=False) # Check queue
                    
                    current_position = self.master_board.get_state()[0] # Update joints current position

                    trajectory.set_initial_conditions(q, current_time) # Set initial condition for trajectory

                else:
                    if self._terminating:
                        self._running = False

                ic_time_array[cpt] =  time.perf_counter() - time_before_initial_condition
            else:
                # Follow current trajectory
                q, q_prime = trajectory.step(current_time) 
                timeout = self.master_board.set_reference(q, q_prime)

                if trajectory.Done():
                    if self._terminating:
                        self.terminate()
                    else:
                        trajectory = None

            #Shared position. This allows InputTask to read the current position from a shared array
            current_position = self.master_board.get_actual_state()[0]
            with self.shared_position.get_lock():
                self.shared_position[:] = current_position

            # Update current reference to each joint.
            timeout = self.master_board.track_reference()
            self.plot.add_time(current_time)
            
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

        #Reset prio
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

        for i in range(config.num_joints // 2):
            self.plot.create_axis([f"Joint {2 * i}, Position Reference", f"Joint {2 * i}, Position Reading"])
            self.plot.create_axis([f"Joint {2 * i + 1}, Position Reference", f"Joint {2 * i + 1}, Position Reading"])
            self.plot.plot()

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
            self._terminating = True
        elif input_message == "Position":
            self.print_current_position(q)

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

    def terminate(self):
        print(f"Master board task with pid: {os.getpid()} terminated")
        self._running = False


shared_position = multiprocessing.Array(ctypes.c_double, 8)

standard_input = sys.stdin.fileno()

trajectory_queue = multiprocessing.Queue()
message_queue = multiprocessing.Queue()

input_task = InputTask(trajectory_queue, message_queue, shared_position, standard_input)
master_board_task = MasterBoardTask(trajectory_queue, message_queue, shared_position)

input_thread = multiprocessing.Process(target = input_task.run, daemon = True)
input_thread.start()

master_board_task.run()

os._exit(0)