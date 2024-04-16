#!/usr/bin/env python3
# coding: utf-8

import struct
import time
import serial
import threading


# V3.2.3
class Transbot(object):
    __uart_state = 0

    def __init__(self, com="/dev/ttyTHS1", delay=.002, arm_offset=(0, 0, 0), debug=False):
        # com = "COM30"
        # com="/dev/ttyTHS1"
        # com="/dev/ttyUSB0"
        # com="/dev/ttyAMA0"

        self.ser = serial.Serial(com, 115200)

        # Defined variable
        self.__delay_time = delay
        self.__arm_offset = arm_offset
        self.__debug = debug

        self.__RECEIVE_STATE = 0x00
        self.FUNC_SET_PID = 0x01
        self.FUNC_MOTION = 0x02
        self.FUNC_PWM_SERVO = 0x03
        self.FUNC_RGB = 0x04
        self.FUNC_RGB_EFFECT = 0x05
        self.FUNC_BEEP = 0x06
        self.FUNC_BIG_LED = 0x07
        self.FUNC_AUTO_REPORT = 0x08
        self.FUNC_MOTOR = 0x09
        self.FUNC_SPEED_LIMIT = 0x0B
        self.FUNC_IMU_YAW = 0x0C
        self.FUNC_CAR_RUN = 0x0D

        self.FUNC_UART_SERVO = 0x20
        self.FUNC_UART_SERVO_ID = 0x21
        self.FUNC_UART_SERVO_TORQUE = 0x22
        self.FUNC_ARM_CTRL = 0x23

        self.FUNC_REQUEST_DATA = 0x50
        self.FUNC_VERSION = 0x51

        self.FUNC_RESET_FLASH = 0xA0

        self.__ax = 0
        self.__ay = 0
        self.__az = 0
        self.__gx = 0
        self.__gy = 0
        self.__gz = 0
        self.__velocity = 0
        self.__angular = 0

        self.__read_id = 0
        self.__read_val = 0

        self.__version_H = 0
        self.__version_L = 0
        self.__version = 0

        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0

        self.__imu_state = False

        self.__battery_voltage = 0

        if self.__debug:
            print("cmd_delay=" + str(self.__delay_time) + "s")

        if self.ser.isOpen():
            print("Serial Opened! Baudrate=115200")
        else:
            print("Serial Open Failed!")

    def __del__(self):
        self.ser.close()
        self.__uart_state = 0
        print("serial Close!")

    def __parse_data(self, ext_type, ext_data):
        # print("parse_data:", ext_data, ext_type)
        # Make the corresponding analysis according to the type of data frame
        if ext_type == 0x08:
            # print(ext_data)
            # SPEED
            self.__velocity = float(struct.unpack('b', bytearray(ext_data[0:1]))[0]) / 100.0
            self.__angular = float(struct.unpack('h', bytearray(ext_data[1:3]))[0]) / 100.0
            # Accelerated velocity
            # Set scaling factor according to chip manual
            accel_ratio = 1 / 16384.0
            self.__ax = struct.unpack('h', bytearray(ext_data[3:5]))[0] * accel_ratio
            self.__ay = struct.unpack('h', bytearray(ext_data[5:7]))[0] * accel_ratio
            self.__az = struct.unpack('h', bytearray(ext_data[7:9]))[0] * accel_ratio
            # angular velocity
            # Degrees over s goes to radians over s
            gyro_ratio = 1 / 65.5 / (180 / 3.1415926)
            # gyro_ratio = 1 / 16.4 / (180 / 3.1415926) # ±2
            # gyro_ratio = 1 / 32.8 / (180 / 3.1415926)
            self.__gx = struct.unpack('h', bytearray(ext_data[9:11]))[0] * gyro_ratio
            self.__gy = struct.unpack('h', bytearray(ext_data[11:13]))[0] * gyro_ratio
            self.__gz = struct.unpack('h', bytearray(ext_data[13:15]))[0] * gyro_ratio
            # Battery voltage value
            self.__battery_voltage = struct.unpack('B', bytearray(ext_data[15:16]))[0]

        else:
            # Robotic arm
            if ext_type == self.FUNC_UART_SERVO:
                self.__read_id = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__read_val = struct.unpack('h', bytearray(ext_data[1:3]))[0]
            # Version number
            elif ext_type == self.FUNC_VERSION:
                self.__version_H = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                self.__version_L = struct.unpack('B', bytearray(ext_data[1:2]))[0]
                # print("Version:", ext_data)
            # PID
            elif ext_type == self.FUNC_SET_PID:
                self.__RECEIVE_STATE = self.FUNC_SET_PID
                self.__kp1 = struct.unpack('h', bytearray(ext_data[0:2]))[0]
                self.__ki1 = struct.unpack('h', bytearray(ext_data[2:4]))[0]
                self.__kd1 = struct.unpack('h', bytearray(ext_data[4:6]))[0]
                # print("PID:", ext_data, self.__RECEIVE_STATE)
            # Gyroscope adjustment state
            elif ext_type == self.FUNC_IMU_YAW:
                self.__RECEIVE_STATE = self.FUNC_IMU_YAW
                self.__imu_state = struct.unpack('B', bytearray(ext_data[0:1]))[0]
                # print("imu:", ext_data)

    # receive data
    def __receive_data(self):
        while True:
            head1 = bytearray(self.ser.read())[0]
            # If frame header 0 is found, continue looking for frame header 1
            if head1 == 0xff:
                head2 = bytearray(self.ser.read())[0]
                check_sum = 0
                rx_check_num = 0
                # If head0 and head1 match successfully, it means that the frame headers match exactly
                if head2 == 0xfd:
                    # Read data length
                    ext_len = bytearray(self.ser.read())[0]
                    # Start reading type
                    ext_type = bytearray(self.ser.read())[0]
                    # Read data
                    ext_data = []
                    check_sum = ext_len + ext_type
                    data_len = ext_len - 2
                    while len(ext_data) < data_len:
                        # Keep reading data
                        value = bytearray(self.ser.read())[0]
                        ext_data.append(value)
                        # The last number is the check bit
                        if len(ext_data) == data_len:
                            rx_check_num = value
                        else:
                            check_sum = check_sum + value
                    # Sum check processing
                    if check_sum % 256 == rx_check_num:
                        self.__parse_data(ext_type, ext_data)
                    else:
                        if self.__debug:
                            print("check sum error:", ext_len, ext_type, ext_data)

    def __request_data(self, function, param=0):
        cmd = [0xff, 0xfe, 0x05, self.FUNC_REQUEST_DATA, int(function) & 0xff, int(param) & 0xff]
        checksum = sum(cmd, 3) & 0xff
        cmd.append(checksum)
        self.ser.write(cmd)
        if self.__debug:
            print("request:", cmd)
        time.sleep(0.0005)

    def __arm_convert_value(self, s_id, s_angle, s_offset=0):
        value = -1
        if s_id == 7:
            # value = int((3100 - 900) * (180 - s_angle - 0) / (180 - 0) + 900)
            value = int((3100 - 900) * (s_angle - s_offset - 180) / (0 - 180) + 900)
        elif s_id == 8:
            # value = int((3100 - 900) * (270 - s_angle - 0) / (180 - 0) + 900)
            value = int((3100 - 900) * (s_angle - 90 - s_offset - 180) / (0 - 180) + 900)
        elif s_id == 9:
            value = int((3100 - 900) * (s_angle + s_offset - 0) / (180 - 0) + 900)
        return value

    def __arm_convert_angle(self, s_id, s_value, s_offset=0):
        s_angle = -1
        if s_id == 7:
            # s_angle = int(180 - (180 - 0) * (s_value - 900) / (3100 - 900) + 0)
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + s_offset + 0.5)
        elif s_id == 8:
            # s_angle = int(270 - (180 - 0) * (s_value - 900) / (3100 - 900) + 0)
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 90 + s_offset + 0.5)
        elif s_id == 9:
            # s_angle = int((180 - 0) * (s_value - 900) / (3100 - 900) + 0)
            s_angle = int((s_value - 900) * (180 - 0) / (3100 - 900) + 0 - s_offset + 0.5)
        return s_angle

    def create_receive_threading(self):
        try:
            if self.__uart_state == 0:
                name1 = "task_serial_receive"
                task_receive = threading.Thread(target=self.__receive_data, name=name1)
                task_receive.setDaemon(True)
                task_receive.start()
                print("----------------create receive threading--------------")
                self.__uart_state = 1
        except:
            print('---create_receive_threading error!---')
            pass

    def set_pid_param(self, kp, ki, kd, forever=False):
        try:
            state = 0
            if forever:
                state = 0x5F
            cmd = [0xff, 0xfe, 0x0A, self.FUNC_SET_PID]
            if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
                print("PID value must be:[0, 10.00]")
                return
            kp_params = bytearray(struct.pack('h', int(kp * 1000)))
            ki_params = bytearray(struct.pack('h', int(ki * 1000)))
            kd_params = bytearray(struct.pack('h', int(kd * 1000)))
            cmd.append(kp_params[0])
            cmd.append(kp_params[1])
            cmd.append(ki_params[0])
            cmd.append(ki_params[1])
            cmd.append(kd_params[0])
            cmd.append(kd_params[1])
            cmd.append(state)
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("pid:", cmd)
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(.1)
        except:
            print('---set_pid_param error!---')
            pass


    def set_car_motion(self, velocity, angular):
        try:
            cmd = [0xff, 0xfe, 0x06, self.FUNC_MOTION]
            if velocity > 0.45:
                velocity = 0.45
            if velocity < -0.45:
                velocity = -0.45
            if angular > 2:
                angular = 2
            if angular < -2:
                angular = -2
            velocity_params = bytearray(struct.pack('h', int(velocity * 100)))
            angular_params = bytearray(struct.pack('h', int(angular * 100)))
            cmd.append(velocity_params[0])
            cmd.append(angular_params[0])
            cmd.append(angular_params[1])
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("motion:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_car_motion error!---')
            pass

    def set_pwm_servo(self, servo_id, angle):
        try:
            cmd = [0xff, 0xfe, 0x05, self.FUNC_PWM_SERVO]
            if angle > 180:
                angle = 180
            elif angle < 0:
                angle = 0
            cmd.append(int(servo_id) & 0xff)
            cmd.append(int(angle) & 0xff)
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("pwmServo:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_pwm_servo error!---')
            pass


    def set_colorful_lamps(self, led_id, red, green, blue):
        try:
            id = int(led_id) & 0xff
            r = int(red) & 0xff
            g = int(green) & 0xff
            b = int(blue) & 0xff
            cmd = [0xff, 0xfe, 0x07, self.FUNC_RGB, id, r, g, b]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("rgb:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_colorful_lamps error!---')
            pass


    def set_colorful_effect(self, effect, speed=255, parm=255):
        try:
            eff = int(effect) & 0xff
            spe = int(speed) & 0xff
            par = int(parm) & 0xff
            cmd = [0xff, 0xfe, 0x06, self.FUNC_RGB_EFFECT, eff, spe, par]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("rgb_effect:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_colorful_effect error!---')
            pass

    def set_beep(self, on_time):
        try:
            if on_time < 0:
                print("beep input error!")
                return
            value = bytearray(struct.pack('h', int(on_time)))

            cmd = [0xff, 0xfe, 0x05, self.FUNC_BEEP, value[0], value[1]]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("beep:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_beep error!---')
            pass


    def set_floodlight(self, light):
        try:
            if light < 0 or light > 100:
                print("light input error!")
                return

            cmd = [0xff, 0xfe, 0x04, self.FUNC_BIG_LED, int(light) & 0xff]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("led:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_floodlight error!---')
            pass


    def set_auto_report_state(self, enable, forever=False):
        try:
            state1 = 0
            state2 = 0
            if enable:
                state1 = 1
            if forever:
                state2 = 0x5F

            cmd = [0xff, 0xfe, 0x05, self.FUNC_AUTO_REPORT, state1, state2]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("report:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_auto_report_state error!---')
            pass


    def set_motor(self, index, speed):
        try:
            if index < 1 or index > 2 or speed < -100 or speed > 100:
                print("motor index input error!")
                return
            t_speed = bytearray(struct.pack('h', int(speed)))

            cmd = [0xff, 0xfe, 0x06, self.FUNC_MOTOR, int(index), t_speed[0], t_speed[1]]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("motor:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_motor error!---')
            pass


    def set_speed_limit(self, line_limit, angular_limit, forever=False):
        try:
            state = 0
            if forever:
                state = 0x5F
            t_line_limit = int(line_limit * 100)
            t_angular_limit = int(angular_limit * 100)
            if t_line_limit < 0 or t_line_limit > 20:
                print("line speed limit input error!")
                return
            if t_angular_limit < 0 or t_angular_limit > 100:
                print("angular speed limit input error!")
                return

            cmd = [0xff, 0xfe, 0x06, self.FUNC_SPEED_LIMIT, t_line_limit, t_angular_limit, state]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("limit:", cmd)
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(.01)
        except:
            print('---set_speed_limit error!---')
            pass


    def set_imu_adjust(self, enable, forever=False):
        try:
            state1 = 0
            state2 = 0
            if enable:
                state1 = 1
            if forever:
                state2 = 0x5F

            cmd = [0xff, 0xfe, 0x05, self.FUNC_IMU_YAW, state1, state2]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("imu:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_imu_adjust error!---')
            pass


    def set_car_run(self, speed):
        try:
            if speed < -0.45 or speed > 0.45:
                print("car_run input error:", speed)
                return
            t_speed = bytearray(struct.pack('b', int(speed * 100)))

            cmd = [0xff, 0xfe, 0x04, self.FUNC_CAR_RUN, t_speed[0]]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("car_run:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_car_run error!---')
            pass


    def set_uart_servo(self, servo_id, pulse_value, run_time=500):
        try:
            if servo_id < 1 or pulse_value < 96 or pulse_value > 4000 or run_time < 0:
                print("set uart servo input error")
                return
            if run_time > 2000:
                run_time = 2000
            if run_time < 0:
                run_time = 0
            s_id = int(servo_id) & 0xff
            value = bytearray(struct.pack('h', int(pulse_value)))
            r_time = bytearray(struct.pack('h', int(run_time)))

            cmd = [0xff, 0xfe, 0x08, self.FUNC_UART_SERVO, s_id, value[0], value[1], r_time[0], r_time[1]]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo:", cmd)
            time.sleep(self.__delay_time)
            # time.sleep(.008)
        except:
            print('---set_uart_servo error!---')
            pass


    def set_uart_servo_angle(self, s_id, s_angle, run_time=500):
        try:
            if s_id == 7:
                if 0 <= s_angle <= 225:
                    # value = int((3100 - 900) * (180 - s_angle - 0) / (180 - 0) + 900)
                    value = self.__arm_convert_value(s_id, s_angle, self.__arm_offset[0])
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_7 set error!")

            elif s_id == 8:
                if 30 <= s_angle <= 270:
                    # value = int((3100 - 900) * (270 - s_angle - 0) / (180 - 0) + 900)
                    value = self.__arm_convert_value(s_id, s_angle, self.__arm_offset[1])
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_8 set error!")

            elif s_id == 9:
                if 30 <= s_angle <= 180:
                    # value = int((3100 - 900) * (s_angle - 0) / (180 - 0) + 900)
                    value = self.__arm_convert_value(s_id, s_angle, self.__arm_offset[2])
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_9 set error!")
        except:
            print('---set_uart_servo_angle error!---')
            pass


    def set_uart_servo_angle_array(self, angle_7, angle_8, angle_9, run_time=500):
        try:
            if 0 <= angle_7 <= 225 and 30 <= angle_8 <= 270 and 30 <= angle_9 <= 180:
                if run_time > 2000:
                    run_time = 2000
                if run_time < 0:
                    run_time = 0
                angle_71 = self.__arm_convert_value(7, angle_7, self.__arm_offset[0])
                angle_81 = self.__arm_convert_value(8, angle_8, self.__arm_offset[1])
                angle_91 = self.__arm_convert_value(9, angle_9, self.__arm_offset[2])
                value7 = bytearray(struct.pack('h', int(angle_71)))
                value8 = bytearray(struct.pack('h', int(angle_81)))
                value9 = bytearray(struct.pack('h', int(angle_91)))
                r_time = bytearray(struct.pack('h', int(run_time)))

                cmd = [0xff, 0xfe, 0x0B, self.FUNC_ARM_CTRL, \
                       value7[0], value7[1], value8[0], value8[1], value9[0], value9[1], \
                       r_time[0], r_time[1]]
                checksum = sum(cmd, 3) & 0xff
                cmd.append(checksum)
                self.ser.write(cmd)
                if self.__debug:
                    print("arm:", cmd)
                    print("value:", angle_71, angle_81, angle_91)
                time.sleep(self.__delay_time)
            else:
                print("angle_7, angle_8, angle_9 input error!")
        except:
            print('---set_uart_servo_angle_array error!---')
            pass


    def set_uart_servo_id(self, servo_id):
        try:
            if servo_id < 1 or servo_id > 250:
                print("servo id input error!")
                return
            cmd = [0xff, 0xfe, 0x04, self.FUNC_UART_SERVO_ID, int(servo_id)]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo_id:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_uart_servo_id error!---')
            pass


    def set_uart_servo_torque(self, enable):
        try:
            if enable > 0:
                on = 1
            else:
                on = 0
            cmd = [0xff, 0xfe, 0x04, self.FUNC_UART_SERVO_TORQUE, on]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo_torque:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_uart_servo_torque error!---')
            pass


    def reset_flash_value(self):
        try:
            cmd = [0xff, 0xfe, 0x04, self.FUNC_RESET_FLASH, 0x5F]
            checksum = sum(cmd, 3) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("flash:", cmd)
            time.sleep(self.__delay_time)
            time.sleep(.1)
        except:
            print('---reset_flash_value error!---')
            pass

    def clear_auto_report_data(self):
        self.__ax, self.__ay, self.__az = 0, 0, 0
        self.__gx, self.__gy, self.__gz = 0, 0, 0
        self.__velocity, self.__angular = 0, 0
        self.__battery_voltage = 0


    def get_uart_servo_value(self, servo_id):
        try:
            if servo_id < 1 or servo_id > 250:
                print("get servo id input error!")
                return
            for i in range(0, 5):
                self.__request_data(self.FUNC_UART_SERVO, int(servo_id) & 0xff)
                time.sleep(.0005)
                if self.__read_id == int(servo_id):
                    read_id = self.__read_id
                    self.__read_id = 0
                    val = self.__read_val
                    self.__read_val = 0
                    return read_id, val
            return -1, -1
        except:
            print('---get_uart_servo_value error!---')
            return -2, -2

    def get_uart_servo_angle(self, s_id):
        try:
            angle = -1
            read_id, value = self.get_uart_servo_value(s_id)
            if s_id == 7 and read_id == 7:
                # angle = int(180 - (180 - 0) * (value - 900) / (3100 - 900) + 0)
                angle = self.__arm_convert_angle(s_id, value, self.__arm_offset[0])
                if angle > 225 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1

            elif s_id == 8 and read_id == 8:
                # angle = int(270 - (180 - 0) * (value - 900) / (3100 - 900) + 0)
                angle = self.__arm_convert_angle(s_id, value, self.__arm_offset[1])
                if angle > 270 or angle < 30:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1

            elif s_id == 9 and read_id == 9:
                # angle = int((180 - 0) * (value - 900) / (3100 - 900) + 0)
                angle = self.__arm_convert_angle(s_id, value, self.__arm_offset[2])
                if angle > 180 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            else:
                if self.__debug:
                    print("read servo error!")
            if self.__debug:
                print("get angle_%d: %d, %d" % (s_id, read_id, value))

            return angle
        except:
            print('---get_uart_servo_angle error!---')
            return -2


    def get_uart_servo_angle_array(self):
        try:
            angle = [-1, -1, -1]
            for i in range(3):
                for x in range(5):
                    temp1 = self.get_uart_servo_angle(i + 7)
                    if temp1 >= 0:
                        angle[i] = temp1
                        break
            return angle
        except:
            print('---get_uart_servo_angle_array error!---')
            return [-2, -2, -2]


    def get_accelerometer_data(self):
        a_x, a_y, a_z = self.__ax, self.__ay, self.__az
        # self.__ax, self.__ay, self.__az = 0, 0, 0
        return a_x, a_y, a_z

    def get_gyroscope_data(self):
        g_x, g_y, g_z = self.__gx, self.__gy, self.__gz
        # self.__gx, self.__gy, self.__gz = 0, 0, 0
        return g_x, g_y, g_z

    def get_motion_data(self):
        val_v = self.__velocity
        val_a = self.__angular
        # self.__velocity, self.__angular = 0, 0
        return val_v, val_a

    def get_battery_voltage(self):
        vol = self.__battery_voltage / 10.0
        # self.__battery_voltage = 0
        return vol

    def get_motion_pid(self):
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0
        self.__RECEIVE_STATE = 0
        for i in range(0, 10):
            self.__request_data(self.FUNC_SET_PID)
            if self.__RECEIVE_STATE == self.FUNC_SET_PID:
                kp = float(self.__kp1 / 1000.0)
                ki = float(self.__ki1 / 1000.0)
                kd = float(self.__kd1 / 1000.0)
                self.__kp1 = 0
                self.__ki1 = 0
                self.__kd1 = 0
                self.__RECEIVE_STATE = 0
                if self.__debug:
                    print("get_motion_pid:{0}, i:{1}".format([kp, ki, kd], i))
                return [kp, ki, kd]
        return [-1, -1, -1]

    def get_imu_state(self):
        self.__RECEIVE_STATE = 0
        self.__imu_state = 0
        for i in range(0, 10):
            self.__request_data(self.FUNC_IMU_YAW)
            if self.__RECEIVE_STATE == self.FUNC_IMU_YAW:
                imu_state = False
                if self.__imu_state == 1:
                    imu_state = True
                self.__imu_state = 0
                self.__RECEIVE_STATE = 0
                if self.__debug:
                    print("get_imu_state:", imu_state)
                return imu_state
        return -1

    def get_version(self):
        if self.__version_H == 0:
            for i in range(0, 10):
                self.__request_data(self.FUNC_VERSION)
                if self.__version_H != 0:
                    val = self.__version_H * 1.0
                    self.__version = val + self.__version_L / 10.0
                    if self.__debug:
                        print("get_version:V{0}, i:{1}".format(self.__version, i))
                    return self.__version
        else:
            return self.__version
        return -1


if __name__ == '__main__':
    # bot = Transbot(com='COM35', debug=True)
    bot = Transbot(debug=True)
    bot.create_receive_threading()
    bot.set_beep(50)
    time.sleep(.1)

    version = bot.get_version()
    print("version=", version)

    try:
        while True:
            # ax, ay, az = bot.get_accelerometer_data()
            # gx, gy, gz = bot.get_gyroscope_data()
            # v, a = bot.get_motion_data()
            # print("v:%.2f, a:%.2f" % (v, a))
            # print(ax, ay, az, gx, gy, gz, v, a)
            # pid = bot.get_motion_pid()
            # print(pid)
            version = bot.get_version()
            print("version=", version)
            time.sleep(.11)
    except KeyboardInterrupt:
        bot.set_car_motion(0, 0)
        pass
    exit(0)
