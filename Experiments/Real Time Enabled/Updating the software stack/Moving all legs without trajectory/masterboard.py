#Standard python modules
import time
import numpy as np

#RASH modules
import config

#ODRI mpdules
import libmaster_board_sdk_pywrap as mbs


class MasterBoard:
    def __init__(self):
        self.robot_if = mbs.MasterBoardInterface("enp4s0")
        self.robot_if.Init()

        self.motors_spi_connected_indexes = []

        num_joints = config.num_joints
        self.motor_positions = np.zeros(num_joints)
        self.motor_velocities = np.zeros(num_joints)

        #Class wide variables from config file
        self.kp = config.kp
        self.kd = config.kd

        self.current_saturation = config.current_saturation

        self.gear_ratio = config.gear_ratio


        #Local variables
        ready = False

        N_SLAVES_CONTROLLED = config.N_SLAVES_CONTROLLED
        dt = config.dt

        #Make sure master board is ready
        for i in range(N_SLAVES_CONTROLLED):
            self.robot_if.GetDriver(i).motor1.SetCurrentReference(0)
            self.robot_if.GetDriver(i).motor2.SetCurrentReference(0)
            self.robot_if.GetDriver(i).motor1.Enable()
            self.robot_if.GetDriver(i).motor2.Enable()
            self.robot_if.GetDriver(i).EnablePositionRolloverError()
            self.robot_if.GetDriver(i).SetTimeout(0) #Fant også at 5 timet ut motorer

            self.robot_if.GetDriver(i).Enable()

        last = time.perf_counter()
        while (not self.robot_if.IsTimeout() and not self.robot_if.IsAckMsgReceived()):
            if ((time.perf_counter() - last) > dt):
                last = time.perf_counter()
                
                self.robot_if.SendInit()

        if self.robot_if.IsTimeout():
            print("Timout while waiting for ack.")
        else:
            for i in range(N_SLAVES_CONTROLLED):
                if self.robot_if.GetDriver(i).IsConnected():
                    self.motors_spi_connected_indexes.append(2 * i)
                    self.motors_spi_connected_indexes.append(2 * i + 1)
        

        while not ready:
            if ((time.perf_counter() - last) > dt):
                last = time.perf_counter()

                self.robot_if.ParseSensorData()

                ready = True
                for i in self.motors_spi_connected_indexes:

                    if not (self.robot_if.GetMotor(i).IsEnabled() and self.robot_if.GetMotor(i).IsReady()):
                        ready = False

                    self.motor_positions[i] = self.robot_if.GetMotor(i).GetPosition()

                self.robot_if.SendCommand()
        print("Done")
    def track_reference(self, q, q_prime):
        #Convert reference to motor space
        pos_ref = q * 9
        vel_ref = q_prime * 9

        if self.robot_if.IsTimeout():
            print("Masterboard timeout detected.")
            print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")
            return True

        self.robot_if.ParseSensorData()

        #Initializing variables for better performance (?)
        kp = self.kp
        kd = self.kd

        current_saturation = self.current_saturation

        for i in self.motors_spi_connected_indexes:
            if i % 2 == 0 and self.robot_if.GetDriver(i // 2).GetErrorCode() == 0xf:
                continue

            if self.robot_if.GetMotor(i).IsEnabled():
                current_position = self.robot_if.GetMotor(i).GetPosition()
                current_velocity = self.robot_if.GetMotor(i).GetVelocity()

                self.motor_positions[i] = current_position
                self.motor_velocities[i] = current_velocity

                pos_err = pos_ref[i] - current_position
                vel_err = vel_ref[i] - current_velocity

                current_ref = kp * pos_err + kd * vel_err

                if (current_ref > current_saturation):
                    current_ref = current_saturation
                if (current_ref < -current_saturation):
                    current_ref = -current_saturation

                self.robot_if.GetMotor(i).SetCurrentReference(current_ref)

        self.robot_if.SendCommand()

        return False

    def get_state(self):
        get_pos = self.motor_positions
        get_vel = self.motor_velocities

        get_pos = get_pos / self.gear_ratio
        get_vel = get_vel / self.gear_ratio

        return get_pos, get_vel

    def terminate(self):
        self.robot_if.Stop()