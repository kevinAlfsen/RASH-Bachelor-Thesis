# coding: utf8

import argparse
import math
import os
import sys
from time import clock

import libmaster_board_sdk_pywrap as mbs

from Trajectory import trajectory
from Plotter import overlay_plot

def example_script(name_interface):

    N_SLAVES = 6  #  Maximum number of controled drivers
    N_SLAVES_CONTROLED = 1  # Current number of controled drivers

    cpt = 0  # Iteration counter
    dt = 0.001  #  Time step
    t = 0  # Current time

    kp = 1  #  Proportional gain
    kd = 0.06  # Derivative gain
    iq_sat = 1.0  # Maximum amperage (A)
    init_pos = [0.0 for i in range(N_SLAVES * 2)]  # List that will store the initial position of motors
    state = 0  # State of the system (ready (1) or not (0))

    t_f = 20

    p_arr = []
    v_arr = []
    p_ref_arr = []
    v_ref_arr = []
    p_err_arr = []
    v_err_arr = []
    curr_arr = []
    t_arr = []

    init_trajectory = True

    # contrary to c++, in python it is interesting to build arrays
    # with connected motors indexes so we can simply go through them in main loop
    motors_spi_connected_indexes = [] # indexes of the motors on each connected slaves

    print("-- Start of example script --")

    os.nice(-20)  #  Set the process to highest priority (from -20 highest to +20 lowest)
    robot_if = mbs.MasterBoardInterface("enp0s31f6")
    robot_if.Init()  # Initialization of the interface between the computer and the master board
    for i in range(N_SLAVES_CONTROLED):  #  We enable each controler driver and its two associated motors
        robot_if.GetDriver(i).motor1.SetCurrentReference(0)
        robot_if.GetDriver(i).motor2.SetCurrentReference(0)
        robot_if.GetDriver(i).motor1.Enable()
        robot_if.GetDriver(i).motor2.Enable()
        robot_if.GetDriver(i).EnablePositionRolloverError()
        robot_if.GetDriver(i).SetTimeout(5)
        robot_if.GetDriver(i).Enable()

    last = clock()

    while (not robot_if.IsTimeout() and not robot_if.IsAckMsgReceived()):
        if ((clock() - last) > dt):
            last = clock()
            robot_if.SendInit()

    if robot_if.IsTimeout():
        print("Timeout while waiting for ack.")
    else:

        # fill the connected motors indexes array
        for i in range(N_SLAVES_CONTROLED):
            if robot_if.GetDriver(i).IsConnected():
                # if slave i is connected then motors 2i and 2i+1 are potentially connected
                motors_spi_connected_indexes.append(2 * i)
                motors_spi_connected_indexes.append(2 * i + 1)

    while ((not robot_if.IsTimeout()) and (clock() < 20)):  # Stop after 15 seconds (around 5 seconds are used at the start for calibration)

        if t > t_f:
            break

        if ((clock() - last) > dt):
            last = clock()
            cpt += 1
            t += dt
            robot_if.ParseSensorData()  # Read sensor data sent by the masterboard

            if (state == 0):  #  If the system is not ready
                state = 1

                # for all motors on a connected slave
                for i in motors_spi_connected_indexes:  # Check if all motors are enabled and ready
                    if not (robot_if.GetMotor(i).IsEnabled() and robot_if.GetMotor(i).IsReady()):
                        state = 0
                    init_pos[i] = robot_if.GetMotor(i).GetPosition()
                    t = 0

            else:  # If the system is ready

                # for all motors on a connected slave
                for i in motors_spi_connected_indexes:

                    if i % 2 == 0 and robot_if.GetDriver(i // 2).GetErrorCode() == 0xf:
                        #print("Transaction with SPI{} failed".format(i // 2))
                        continue #user should decide what to do in that case, here we ignore that motor

                    if robot_if.GetMotor(i).IsEnabled() and i == 0:

                        if init_trajectory:
                            t_f = t + 2
                            t_i = t
                            q_i = robot_if.GetMotor(0).GetPosition()
                            q_f = q_i + 2*math.pi * 9 # 9 = gear ratio
                            traj = trajectory(q_i, q_f, t_i, t_f)
                            init_trajectory = False

                        if t < t_f:
                            ref, v_ref = traj.step(t)

                        position = robot_if.GetMotor(0).GetPosition()
                        velocity = robot_if.GetMotor(0).GetVelocity()
                        
                        p_err = ref - position  # Position error
                        v_err = v_ref - velocity  # Velocity error
                        cur = kp * p_err + kd * v_err  #  Output of the PD controler (amperage)
                        
                        curr_arr.append(cur)

                        if (cur > iq_sat):  #  Check saturation
                            cur = iq_sat
                        if (cur < -iq_sat):
                            cur = -iq_sat

                        p_arr.append(position - init_pos[0])
                        v_arr.append(velocity)
                        p_ref_arr.append(ref - init_pos[0])
                        v_ref_arr.append(v_ref)
                        p_err_arr.append(p_err)
                        v_err_arr.append(v_err)
                        
                        t_arr.append(t)

                        if (position < q_f):
                            #continue
                            robot_if.GetMotor(i).SetCurrentReference(cur)  # Set reference currents

            robot_if.SendCommand()  # Send the reference currents to the master board

    overlay_plot(p_arr, v_arr, p_ref_arr, v_ref_arr, p_err_arr, v_err_arr, t_arr, curr_arr, kp, kd)


    robot_if.Stop()  # Shut down the interface between the computer and the master board

    

    if robot_if.IsTimeout():
        print("Masterboard timeout detected.")
        print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")

    print("-- End of example script --")


def main():
    parser = argparse.ArgumentParser(description='Example masterboard use in python.')
    parser.add_argument('-i',
                        '--interface',
                        required=True,
                        help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')

    example_script(parser.parse_args().interface)


if __name__ == "__main__":
    main()


