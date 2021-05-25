
#Standard modules
import time
import matplotlib.pyplot as plt
import os
import numpy as np

#RASH modules
import masterboard
import config
import trajectory as traj

#Local variables
_running = True
timeout = False

dt = config.dt
current_time = 0
duration = 100
time_per_timestep_array = []
cycle_time_array = []

trajectory = None
q_f = np.zeros(8)

#Set priority to max and use a realtime scheduler
pid = os.getpid()
sched = os.SCHED_FIFO
param = os.sched_param(70)

os.sched_setscheduler(pid, sched, param)

master_board = masterboard.MasterBoard()
q, q_prime = master_board.get_state()

direction = True

#Timer init
cycle_time = 0
program_start = time.perf_counter()
timestep_start = time.perf_counter()
timestep_end = time.perf_counter()
cycle_start = time.perf_counter()
while(_running and not timeout):

    if trajectory == None:
        #No trajectory, generate a new one with proper direction
        if direction:
            final_position = np.pi/2
            direction = False
        else:
            final_position = -np.pi/2
            direction = True

        #Final position
        q_f.fill(final_position)
        trajectory = traj.Trajectory(q_f)

        #Initial position
        current_position = master_board.get_state()[0]
        trajectory.set_initial_conditions(current_position, current_time)
    else:
        #Follow current trajectory
        q, q_prime = trajectory.step(current_time)
        timeout = master_board.set_reference(q, q_prime)

        if trajectory.Done():
            trajectory = None


    timeout = master_board.track_reference()


    #Terminate if duration has passed
    if (time.perf_counter() - program_start > duration):
        _running = False

    cycle_time = time.perf_counter() - cycle_start
    cycle_time_array.append(cycle_time)

    if cycle_time >= dt:
        #print(f"Cycle time too large: {cycle_time}")
        cycle_time = dt

    time.sleep(dt - cycle_time)
    cycle_start = time.perf_counter()

    timestep_end = cycle_start
    time_per_timestep_array.append(timestep_end - timestep_start)
    timestep_start = cycle_start

    current_time = current_time + dt


master_board.terminate()


#Reset prio
sched = os.SCHED_OTHER
param = os.sched_param(os.sched_get_priority_min(sched))
os.sched_setscheduler(pid, sched, param)

print(f"Set priority to : {param}")

#Plot
plt.plot(time_per_timestep_array)
plt.show()
plt.plot(cycle_time_array)
plt.show()
