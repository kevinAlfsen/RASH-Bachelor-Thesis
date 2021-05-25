
#Standard modules
import time
import matplotlib.pyplot as plt
import os
import numpy as np
import threading
import queue

#RASH modules
import masterboard
import config
import trajectory as trj

class InputTask:
    def __init__(self, trajectory_queue):
        self.trajectory_queue = trajectory_queue


        self._running = True

    def run(self):
        print(f"Input task started with pid: {os.getpid()} and priority: {os.sched_getparam(os.getpid())}")


        trajectory = None

        while(self._running):
            input_string = input(">")

            try:
                q_f = float(input_string)
                trajectory = trj.Trajectory([q_f,q_f,q_f,q_f,q_f,q_f,q_f,q_f])
                
                if trajectory.is_valid():
                    self.trajectory_queue.put(trajectory)
                else:
                    print("Trajectory Invalid")

            except ValueError:
                print(f"Wrong input : {input_string}")

            if "exit" in input_string:
                self.terminate()

        print(f"Input task with pid: {os.getpid()} terminated")
    def terminate(self):
        self._running = False


class MasterBoardTask:
    def __init__(self, trajectory_queue):
        #Local variables
        self._running = True

        self.trajectory_queue = trajectory_queue

        self.master_board = masterboard.MasterBoard()

    def run(self):
        print(f"Master board task started with pid: {os.getpid()} and priority: {os.sched_getparam(os.getpid())}")
        #Set priority to max and use a realtime scheduler
        pid = os.getpid()

        sched = os.SCHED_FIFO
        param = os.sched_param(70)

        os.sched_setscheduler(pid, sched, param)


        print(f"Master board task with pid: {os.getpid()} changed to priority: {os.sched_getparam(os.getpid())}")

        timeout = False

        dt = config.dt

        duration = 20
        current_time = 0
        cpt = 0
        time_per_timestep_array = np.zeros(int(duration / dt))
        cycle_time_array = np.zeros(int(duration / dt))

        trajectory = None

        q, q_prime = self.master_board.get_state()

        #Timer init
        cycle_time = 0
        program_start = time.perf_counter()
        timestep_start = time.perf_counter()
        timestep_end = time.perf_counter()
        cycle_start = time.perf_counter()
        while(self._running and not timeout):

            if trajectory == None:
                #No trajectory, generate a new one with proper direction
                try:
                    trajectory = self.trajectory_queue.get(block=False)

                    current_position = self.master_board.get_state()[0]
                    trajectory.set_initial_conditions(current_position, current_time)
                except queue.Empty:
                    trajectory = None

            else:
                #Follow current trajectory
                q, q_prime = trajectory.step(current_time)
                timeout = self.master_board.set_reference(q, q_prime)

                if trajectory.Done():
                    trajectory = None


            timeout = self.master_board.track_reference()
            

            cycle_time = time.perf_counter() - cycle_start
            cycle_time_array[cpt] = cycle_time

            if cycle_time >= dt:
                cycle_time = dt

            time.sleep(dt - cycle_time)
            cycle_start = time.perf_counter()

            timestep_end = cycle_start
            time_per_timestep_array[cpt] = timestep_end - timestep_start
            timestep_start = cycle_start

            current_time = current_time + dt
            cpt = cpt + 1

            if (cpt >= int(duration / dt)):
                self._running = False


        self.master_board.terminate()

        #Reset prio
        sched = os.SCHED_OTHER
        param = os.sched_param(os.sched_get_priority_min(sched))
        os.sched_setscheduler(pid, sched, param)

        print(f"Master board task with pid: {os.getpid()} changed to priority: {os.sched_getparam(os.getpid())}")

        plt.plot(time_per_timestep_array)
        plt.suptitle("Time per timestep")
        plt.xlabel('Time [ms]')
        plt.ylabel('Elapsed time [s]')
        plt.grid()
        plt.show()

        plt.plot(cycle_time_array)
        plt.suptitle("Cycle time")
        plt.xlabel('Time [ms]')
        plt.ylabel('Elapsed time [s]')
        plt.grid()
        plt.show()
        
    def terminate(self):
        print(f"Master board task with pid: {os.getpid()} terminated")
        self._running = False



trajectory_queue = queue.Queue()
input_task = InputTask(trajectory_queue)
master_board_task = MasterBoardTask(trajectory_queue)

input_thread = threading.Thread(target = input_task.run, daemon = True)
input_thread.start()

master_board_task.run()

os._exit(0)