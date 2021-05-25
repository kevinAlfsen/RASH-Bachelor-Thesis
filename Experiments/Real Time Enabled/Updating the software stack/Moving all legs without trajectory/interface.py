
#Standard modules
import time
import matplotlib.pyplot as plt
import os
import numpy as np

#RASH modules
import masterboard
import config

#Local variables
_running = True
timeout = False

dt = config.dt
duration = 100
time_per_timestep_array = []

#Set priority to max and use a realtime scheduler
pid = os.getpid()
sched = os.SCHED_FIFO
param = os.sched_param(70)

os.sched_setscheduler(pid, sched, param)

master_board = masterboard.MasterBoard()
q, q_prime = master_board.get_state()

direction = True

print(f"Set priority to : {param}")


#Timer init
cycle_time = 0
program_start = time.perf_counter()
timestep_start = time.perf_counter()
timestep_end = time.perf_counter()
cycle_start = time.perf_counter()
while(_running and not timeout):
    if direction:
        q = q + 0.0025
        if (q > np.pi/2).any():
            direction = False
    else:
        q = q - 0.0025
        if (q < -np.pi/2).any():
            direction = True

    timeout = master_board.track_reference(q, q_prime)
    

    #Terminate if duration has passed
    if (time.perf_counter() - program_start > duration):
        _running = False

    cycle_time = time.perf_counter() - cycle_start
    time.sleep(dt - cycle_time)
    cycle_start = time.perf_counter()

    timestep_end = cycle_start
    time_per_timestep_array.append(timestep_end - timestep_start)
    timestep_start = cycle_start


master_board.terminate()


#Reset prio
sched = os.SCHED_OTHER
param = os.sched_param(os.sched_get_priority_min(sched))
os.sched_setscheduler(pid, sched, param)

print(f"Set priority to : {param}")

#Plot
plt.plot(time_per_timestep_array)
plt.suptitle("Time per timestep")
plt.xlabel('Time [ms]')
plt.ylabel('Elapsed time [s]')
plt.grid()
plt.show()
