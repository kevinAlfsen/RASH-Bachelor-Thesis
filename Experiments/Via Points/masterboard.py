#Standard python modules
import time
import numpy as np

#RASH modules
import config

#ODRI mpdules
import libmaster_board_sdk_pywrap as mbs


class MasterBoard:
    def __init__(self):
        #self.plot = plot

        self.robot_if = mbs.MasterBoardInterface("enp4s0")
        self.robot_if.Init()

        self.motors_spi_connected_indexes = []

        num_joints = config.num_joints
        self.num_joints = num_joints

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
            self.robot_if.GetDriver(i).SetTimeout(0)
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

        ready_wait_time = 0
        while not ready:
            if ((time.perf_counter() - last) > dt):
                last = time.perf_counter()
                ready_wait_time = ready_wait_time + dt

                self.robot_if.ParseSensorData()

                ready = True
                for i in self.motors_spi_connected_indexes:

                    if not (self.robot_if.GetMotor(i).IsEnabled() and self.robot_if.GetMotor(i).IsReady()):
                        ready = False

                    self.motor_positions[i] = self.robot_if.GetMotor(i).GetPosition()

                self.robot_if.SendCommand()

        self.q = np.copy(self.motor_positions)
        self.q_prime = np.copy(self.motor_velocities)

        self.zero_calibration = np.zeros(8)
        self.floor_calibration = np.zeros(8)

        self.imu_data = np.zeros(3)
        self.imu_correction = np.array([np.pi, 0, 0]) #Our imu is turned upside down
        
        if ready_wait_time < 1:
            self.get_calibrations_from_file()

        else:
            print("Robot needs zero calibration")
            print("To calibrate, start calibrate.py")

            print("Using previous calibration")

            self.get_calibrations_from_file()
            
        print(f"Zero Calibration set to : {np.degrees(self.zero_calibration)} [DEG]")
        print(f"Floor Calibration set to : {np.degrees(self.floor_calibration)} [DEG]")
        
        #print("Master board initiated")

    def track_reference(self, plot=None, limp=False):
        #Convert reference to motor space
        pos_ref = self.q
        vel_ref = self.q_prime

        self.robot_if.ParseSensorData()

        for i in range(3):
            self.imu_data[i] = self.robot_if.imu_data_attitude(i)

        #Initializing variables for better performance (?)
        kp = self.kp
        kd = self.kd

        #plot = self.plot

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

                #Current here refers to the motor amperage
                current_ref = kp * pos_err + kd * vel_err

                if plot != None:
                    plot.add_data(f"Joint {i}, Position Reference", pos_ref[i])
                    plot.add_data(f"Joint {i}, Position Reading", current_position)
                    plot.add_data(f"Joint {i}, Velocity Reference", vel_ref[i])
                    plot.add_data(f"Joint {i}, Velocity Reading", current_velocity)

                if (current_ref > current_saturation):
                    current_ref = current_saturation
                if (current_ref < -current_saturation):
                    current_ref = -current_saturation

                if limp:
                    current_ref = 0

                self.robot_if.GetMotor(i).SetCurrentReference(current_ref)

        self.robot_if.SendCommand()

        if self.robot_if.IsTimeout():
            print("Masterboard timeout detected.")
            print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")
            return True

        return False

    def maintain_communication(self):
        self.robot_if.ParseSensorData()

        for i in self.motors_spi_connected_indexes:
            if i % 2 == 0 and self.robot_if.GetDriver(i // 2).GetErrorCode() == 0xf:
                continue

            if self.robot_if.GetMotor(i).IsEnabled():
                current_position = self.robot_if.GetMotor(i).GetPosition()
                current_velocity = self.robot_if.GetMotor(i).GetVelocity()

                self.motor_positions[i] = current_position
                self.motor_velocities[i] = current_velocity

                self.robot_if.GetMotor(i).SetCurrentReference(0)

        self.robot_if.SendCommand()

        if self.robot_if.IsTimeout():
            print("Masterboard timeout detected.")
            print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")
            return True

        return False

    def get_state(self):
        gear_ratio = self.gear_ratio

        get_pos = self.motor_positions
        get_vel = self.motor_velocities

        get_pos = get_pos / gear_ratio
        get_vel = get_vel / gear_ratio

        get_pos = get_pos - self.zero_calibration - self.floor_calibration

        return get_pos, get_vel

    def get_actual_state(self):
        '''
        Converts position and velocites to output angles.
        Does not subtract relevant offsets and calibrations
        '''
        gear_ratio = self.gear_ratio

        get_pos = self.motor_positions
        get_vel = self.motor_velocities

        get_pos = get_pos / gear_ratio
        get_vel = get_vel / gear_ratio

        return get_pos, get_vel


    def set_reference(self, q, q_prime):
        gear_ratio = self.gear_ratio

        pos_ref = q
        vel_ref = q_prime

        pos_ref = pos_ref + self.zero_calibration + self.floor_calibration

        pos_ref = pos_ref * gear_ratio
        vel_ref = vel_ref * gear_ratio

        self.q = pos_ref
        self.q_prime = vel_ref

    def get_calibrations_from_file(self):
        num_joints = self.num_joints

        zero_calibration = np.zeros(num_joints)
        floor_calibration = np.zeros(num_joints)

        try:
            zero_file = open("zero_calibration.txt", "r")
            zero_string = zero_file.read()
            zero_numbers = zero_string.split(",")

            for i in range(num_joints):
                zero_calibration[i] = float(zero_numbers[i])
        
        except FileNotFoundError:
            zero_calibration = np.zeros(8)

        try:
            floor_file = open("floor_calibration.txt", "r")
            floor_string = floor_file.read()
            floor_numbers = floor_string.split(",")

            for i in range(num_joints):
                floor_calibration[i] = float(floor_numbers[i])
        
        except FileNotFoundError:
            floor_calibration = np.zeros(8)
        
        self.zero_calibration = zero_calibration
        self.floor_calibration = floor_calibration

    def get_attitude(self):
        self.imu_data[0] = self.imu_data[0] + np.pi

        if self.imu_data[0] > np.pi:
            self.imu_data[0] = - 2 * np.pi + self.imu_data[0]


        return self.imu_data

    def terminate(self):
        self.robot_if.Stop()