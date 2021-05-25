#config.py
#Class containing all configuration variables
#Used to declutter modules that share config variables
import config
from config import Config

#parser.py
import parser
from parser import ParseError

#trajectory.py
#Holds final and initial positions and calculates the interpolation between them
from trajectory import Trajectory

#plotter.py
#Holds arrays with data that will be plotted as well as methods for plotting with matplotlib
#Makes potting easier
from plotter import Plotter

#masterboard.py
#Holds a reference to the masterboard sdk
#Contains methods for setting the reference position of the motors and tracking it with a PD controller
from masterboard import MasterBoard

#time counter, used to count time
from time import perf_counter

#Queue is used to transfer data between threads
#Empty is an exception thrown when trying to get from an empty queue
from queue import Queue, Empty

#Threading is used so that user input can be prompted without pausing the master board communication
from threading import Thread, settrace #, Lock #Lock might be useful later, if GIL is not in place

import numpy as np

class MasterBoardTask:
    def __init__(self, trajectory_queue, plotter):
        #Instatiate Master Board and the plotter
        self.plotter = plotter
        self.master_board = MasterBoard(self.plotter)

        #Initiate global variables
        self.trajectory_queue = trajectory_queue
        self.trajectory = None

        self.num_joints = Config.num_joints

        self.encoder_readings = []

        self._running = True
        self._should_print = False
        #Initiate plots

        for i in range(Config.num_joints):
            self.plotter.create_plot(f"Joint {i}, Position Reference", "Position Reference [rad]")
            self.plotter.create_plot(f"Joint {i}, Position Reading", "Position Reading [rad]")
            #self.plotter.create_plot(f"Joint {i}, Position Error", "Position Error [rad]")

    def run(self):
        print("Master board task started")

        #Initiate local variables
        t = 0

        timeout = False

        last = perf_counter()

        q = self.master_board.get_state()[0]
        q_prime = 0

        #Main loop
        #Executes every time step untill keyboardinterrupt or master_board sets timeout = True
        while(not timeout and self._running):
            elapsed_time = perf_counter() - last

            if (elapsed_time >= Config.dt): #Enter this if statement every timestep
                last = perf_counter()
                t += Config.dt

                if self.trajectory != None:
                    #Currently following a trajectory, just get the new values and send them
                    q, q_prime = self.trajectory.step(t)
                    self.master_board.set_reference(q, q_prime)

                    position_reading, velocity_reading = self.master_board.get_state()

                    for i in range(Config.num_joints):
                        self.plotter.add_data(f"Joint {i}, Position Reference", np.degrees(q[i]))
                        self.plotter.add_data(f"Joint {i}, Position Reading", np.degrees(position_reading[i]))
                        #self.plotter.add_data(f"Joint {i}, Position Error", np.degrees(q[i] - position_reading[i]))
                    self.plotter.add_time(t)


                    #Then check if that was the last step, i.e. the trajectory is finished
                    if self.trajectory.Done():
                        #If the trajectory is finished, set it to None so that the next pass will fetch the next trajectory
                        #print("Trajectory finished, waiting for next...")
                        
                        #self.set_points.append(q[0])
                        self.encoder_readings.append(position_reading[0])
                        self.trajectory = None

                else:
                    #Not currently following a trajectory, so try to fetch the next one from the queue

                    position_reading, velocity_reading = self.master_board.get_state()

                    for i in range(Config.num_joints):
                        self.plotter.add_data(f"Joint {i}, Position Reference", np.degrees(q[i]))
                        self.plotter.add_data(f"Joint {i}, Position Reading", np.degrees(position_reading[i]))
                        #self.plotter.add_data(f"Joint {i}, Position Error", np.degrees(q[i] - position_reading[i]))
                    self.plotter.add_time(t)

                    try:
                        self.trajectory = self.trajectory_queue.get(block=False)
                        self.trajectory.set_initial_conditions(q, t) #Last reference sent and current time will be the intitial conditions for the trajectory
                    except Empty:
                        self.trajectory = None

                #If a new reference was set, this will make the masterboard track that reference
                #If not, it will keep tracking its previous reference
                #This needs to be called every pass, even though there is no new trajectory, in case the actual position of the joints are changed, e.g. by gravity
                timeout = self.master_board.follow_trajectory()

            #Should print is set by InputTask whenever "Position" is input to the terminal
            #This is in an elif block, so that it doesn't increase the duration of a step
            elif self._should_print:
                self.print_current_position(q)

        #shut down the board correctly if the while loop breaks for any reason
        self.master_board.terminate()

        print("\nENCODER READINGS\n")
        for value in self.encoder_readings:
            output = np.degrees(value)
            output = str(output)
            output = output.replace(".", ",")
            print(output)
        print()

        for i in range(Config.num_joints):
            self.plotter.create_axis([f"Joint {i}, Position Reference", f"Joint {i}, Position Reading"])
            #self.plotter.create_axis(f"Joint {i}, Position Error")
            self.plotter.plot()

    def print_current_position(self, current_pos_ref):
        #Prints the current reference and position of all the motors
        #Meant to be called by the InputTask through the use of synchronized bools
        np.set_printoptions(suppress=True)
        print(f"\nPosition Reference: \n{np.degrees(current_pos_ref).round(5)}\n")
        print(f"Current Position: \n{self.master_board.get_state()[0]}")
        print(f"Position Error: \n{np.degrees(current_pos_ref.round(5) - self.master_board.get_state()[0]).round(5)}")
        self._should_print = False

    def terminate(self):
        self._running = False

    def set_print(self):
        self._should_print = True

class InputTask:
    def __init__(self, trajectory_queue, master_board_task):
        self.trajectory_queue = trajectory_queue
        self.master_board_task = master_board_task

        self.generated = False

    def run(self):
        print("Input task started")
        print("[q = 1,2,3,4,5,6,7,8 or]\n[q0 = pi/2]\n\n")

        #Thread is daemonic, so this loop will only stop when the main thread is stopped
        while(True):
            try:
                #Waits for an input.
                #If the input is joint angles, create a trajectory and add it to the queue
                q_f = parser.parse_input()

                trajectory = Trajectory(q_f)

                if trajectory.is_valid():
                    self.trajectory_queue.put(trajectory)
                else:
                    print("Trajectory not valid")
                    #trajectory_queue.join()
                    #self.master_board.terminate()

                    #Should stop the robot where it is, but not shut down the master board

                #If parser detects one of these specified inputs, the specified action will happen
            except ParseError as e:
                if e.msg == "Exit":
                    self.master_board_task.terminate()
                if e.msg == "Position":
                    self.master_board_task.set_print()
                if e.msg == "Calibrate":
                    if self.generated == False:
                        print("Calibration initated, type c to go to next point")
                        calibration_generator = config.Calibration_parameters(0)
                        self.generated = True

                    else:
                        try:
                            trajectory = Trajectory(next(calibration_generator))
                            self.trajectory_queue.put(trajectory)
                        except StopIteration:
                            print("Calibration Done")
                            self.generated = False

                    #for parameters in config.Calibration_parameters(0):
                    #    trajectory = Trajectory(parameters, Config.clibration_delay)
                    #    self.trajectory_queue.put(trajectory)

#Locked object to communicate between threads
#As long as the GIL is implented, this is not needed ?
#
#class SharedFlags:
#    def __init__(self):
#        self.should_print = False
#        self.flag_lock = Lock()
#
#    def set_print(self):
#        with self.flag_lock:
#            self.should_print = True
#
#    def get_print(self):
#        with self.flag_lock:
#            return self.should_print

trajectory_queue = Queue()
plotter = Plotter("Error Analysis")

master_board_task = MasterBoardTask(trajectory_queue, plotter)



input_task = InputTask(trajectory_queue, master_board_task)
input_thread = Thread(target=input_task.run, daemon=True)
input_thread.start()

master_board_task.run()

exit()


