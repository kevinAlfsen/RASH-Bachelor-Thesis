import masterboard
import threading
import time
import config
import numpy as np

class InputTask:
    def __init__(self):
        self.zero_calibration_flag = False
        self.floor_calibration_flag = False
        self.limp_flag = True
        self.input_flag = True
        self.print_startup_message = True
        self.set_calibration = False

        self.zero_offset = 0
        self.floor_calibration_value = 0


        self.floor_position = 0
        self.desired_zero_position = 0

    def run(self):
        dt = config.dt 

        while (1):
            if self.input_flag:
                if self.print_startup_message:
                    print()
                    print("------- Available Calibration Options: ---------")
                    print("Zero -> Find new zero after master board reboot")
                    print("Floor -> Find distance from floor to wanted zero position")
                    print("Ctrl + C -> exit")
                    print_startup_message = False

                input_string = input(">")

                if input_string == "Zero":
                    self.input_flag = False

                    self.limp_flag = True
                    print()
                    print("Zero calibration initiated")
                    print("Make sure all legs are touching ground")
                    
                    self.wait_for_confirmation()
                    self.zero_calibration_flag = True

                    while (self.zero_calibration_flag):
                        time.sleep(dt)

                    zero_offset = self.zero_offset

                    file = open("zero_calibration.txt", "w")
                    for value in zero_offset:
                        file.write(f"{value},")
                    file.close()

                    print(f"Zero calibration set to {zero_offset}")
                    print()

                    print("!Robot will move!")
                    self.wait_for_confirmation()

                    self.limp_flag = False
                    self.set_calibration = True

                    print("Moved to zero position")

                    self.reset()

                if input_string == "Floor":
                    self.input_flag = False

                    self.limp_flag = True
                    print()
                    print("Floor calibration initiated")
                    print("Make sure all legs are touching the ground\n")

                    self.wait_for_confirmation()
                    self.floor_calibration_flag = True

                    while(self.floor_calibration_flag):
                        time.sleep(dt)

                    self.floor_position = self.floor_calibration_value

                    print("Place brackets beneath legs, to put them in the desired zero position")
                    self.wait_for_confirmation()
                    self.floor_calibration_flag = True

                    while(self.floor_calibration_flag):
                        time.sleep(dt)

                    self.desired_zero_position = self.floor_calibration_value

                    floor_offset = self.desired_zero_position - self.floor_position

                    # Write to file
                    file = open("floor_calibration.txt", "w")
                    for value in floor_offset:
                        file.write(f"{value},")
                    file.close()

                    print(f"Floor calibraiton set to : {floor_offset}")
                    print()

                    print("!Robot will move!")
                    self.wait_for_confirmation()

                    self.limp_flag = False
                    self.set_calibration = True

                    print("Moved to zero position")

                    self.reset()

    def wait_for_confirmation(self):
        input("Press enter to continue")
        print()

    def reset(self):
        self.input_flag = True
        self.print_startup_message = True

        

input_task = InputTask()
input_thread = threading.Thread(target=input_task.run, daemon=True)

master_board = masterboard.MasterBoard()

input_thread.start()

dt = config.dt

timeout = False
running = True

cycle_start = time.perf_counter()
while (not timeout and running):

    if input_task.zero_calibration_flag:
        input_task.zero_offset = master_board.get_actual_state()[0]
        input_task.zero_calibration_flag = False

        master_board.set_reference(np.zeros(8), np.zeros(8))

    elif input_task.floor_calibration_flag:
        # Floor calibration initiated, make sure the legs are touching the ground
        input_task.floor_calibration_value = master_board.get_actual_state()[0]
        input_task.floor_calibration_flag = False

        # Set position reference to zero for all joints
        master_board.set_reference(np.zeros(8), np.zeros(8))

    elif input_task.set_calibration:
        input_task.set_calibration = False
        master_board.get_calibrations_from_file()

        master_board.set_reference(np.zeros(8), np.zeros(8))

    timeout = master_board.track_reference(limp=input_task.limp_flag)

    cycle_time = time.perf_counter() - cycle_start
    if cycle_time > dt:
        cycle_time = dt
    time.sleep(dt - cycle_time)
    cycle_start = time.perf_counter()
 