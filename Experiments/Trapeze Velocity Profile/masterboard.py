import os
from time import perf_counter

from config import Config

import numpy as np

import libmaster_board_sdk_pywrap as mbs

class MasterBoard:
    def __init__(self, plotter):
        #Set instance specific variables
        self.plotter = plotter

        self.robot_if = mbs.MasterBoardInterface("enp4s0")
        self.robot_if.Init()

        self.motors_spi_connected_indexes = []

        self.motor_positions = np.array([0.0 for i in range(Config.N_SLAVES * 2)])
        self.motor_velocities = np.array([0.0 for i in range(Config.N_SLAVES * 2)])

        #Set initiation specific variables
        ready = False


        #Initating the master board comunication
        os.nice(-20)

        for i in range(Config.N_SLAVES_CONTROLLED):
            self.robot_if.GetDriver(i).motor1.SetCurrentReference(0)
            self.robot_if.GetDriver(i).motor2.SetCurrentReference(0)
            self.robot_if.GetDriver(i).motor1.Enable()
            self.robot_if.GetDriver(i).motor2.Enable()
            self.robot_if.GetDriver(i).EnablePositionRolloverError()
            self.robot_if.GetDriver(i).SetTimeout(5)
            self.robot_if.GetDriver(i).Enable()

        last = perf_counter()

        while (not self.robot_if.IsTimeout() and not self.robot_if.IsAckMsgReceived()):
            if ((perf_counter() - last) > Config.dt):
                last = perf_counter()
                self.robot_if.SendInit()

        if self.robot_if.IsTimeout():
            print("Timeout while waiting for ack.")
        else:
            for i in range(Config.N_SLAVES_CONTROLLED):
                if self.robot_if.GetDriver(i).IsConnected():
                    self.motors_spi_connected_indexes.append(2 * i)
                    self.motors_spi_connected_indexes.append(2 * i + 1)

        while not ready:
            if ((perf_counter() - last) > Config.dt):
                last = perf_counter()

                self.robot_if.ParseSensorData()

                ready = True
                for i in self.motors_spi_connected_indexes:

                    if not (self.robot_if.GetMotor(i).IsEnabled() and self.robot_if.GetMotor(i).IsReady()):
                        ready = False

                    self.motor_positions[i] = self.robot_if.GetMotor(i).GetPosition()

                self.robot_if.SendCommand()

        self.pos_ref = self.motor_positions
        self.vel_ref = self.motor_velocities


    def follow_trajectory(self):
        if self.robot_if.IsTimeout():
            print("Masterboard timeout detected.")
            print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")
            return True

        self.robot_if.ParseSensorData()

        for i in self.motors_spi_connected_indexes:
            if i % 2 == 0 and self.robot_if.GetDriver(i // 2).GetErrorCode() == 0xf:
                continue

            if self.robot_if.GetMotor(i).IsEnabled():
                self.motor_positions[i] = self.robot_if.GetMotor(i).GetPosition()
                self.motor_velocities[i] = self.robot_if.GetMotor(i).GetVelocity()

                pos_err = self.pos_ref[i] - self.motor_positions[i]
                vel_err = self.vel_ref[i] - self.motor_velocities[i]

                current_ref = Config.kp * pos_err + Config.kd * vel_err #PD-controller

                self.plotter.add_data(f"Joint {i}, Current Reference", current_ref)

                if (current_ref > Config.current_saturation):
                    current_ref = Config.current_saturation
                if (current_ref < -Config.current_saturation):
                    current_ref = -Config.current_saturation

                #if self.motor_positions[i] > Config.joint_limits[0][i] * Config.gear_ratio:
                    #self.pos_ref[i] = Config.joint_limits[0][i] * Config.gear_ratio
                    #self.vel_ref[i] = 0
                #    current_ref = 0
                #elif self.motor_positions[i] < Config.joint_limits[1][i] * Config.gear_ratio:
                    #self.pos_ref[i] = Config.joint_limits[1][i] * Config.gear_ratio
                    #self.vel_ref[i] = 0
                #    current_ref = 0

                self.robot_if.GetMotor(i).SetCurrentReference(current_ref)

        self.robot_if.SendCommand()

        return False

    def set_reference(self, q, q_prime):
        #Takes a position reference and a velocity reference, q and q_prime
        #q and q_prime are lists of size (len(num_joints))
        #Converts them to motor angles and adds all relevant offsets and calibrations

        pos_ref = np.array(q)
        vel_ref = np.array(q_prime)

        pos_ref = pos_ref + Config.zero_calibration
        vel_ref = vel_ref

        pos_ref = pos_ref * Config.gear_ratio
        vel_ref = vel_ref * Config.gear_ratio

        self.pos_ref = pos_ref
        self.vel_ref = vel_ref

    def get_state(self):
        #Converts positions and velocites to output angles
        #Subtracts all relevant offsets and calibration and returns them

        get_pos = self.motor_positions
        get_vel = self.motor_velocities

        get_pos = get_pos / Config.gear_ratio
        get_vel = get_vel / Config.gear_ratio

        get_pos = get_pos - Config.zero_calibration
        get_vel = get_vel

        return get_pos, get_vel

    def terminate(self):
        self.robot_if.Stop()