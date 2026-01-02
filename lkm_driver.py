import serial
import time
import struct

class MotorDriver:
    def __init__(self, 
                 SERIAL_PORT = '/dev/ttyUSB0',
                 BAUDRATE = 115200,
                 MOTOR_ID = 1,
                 REDUCTION_RATIO = 9.0,
                 ENCODER_RESOLUTION_DEG = 0.01):
        
        #connection specifics
        self.SERIAL_PORT = SERIAL_PORT
        self.BAUDRATE = BAUDRATE
        self.MOTOR_ID = MOTOR_ID

        #motor specifics
        self.REDUCTION_RATIO = REDUCTION_RATIO  # The 'i' parameter. IE: MG8008E-i9 <- this last value
        self.ENCODER_RES_DEG = ENCODER_RESOLUTION_DEG # 1 unit = 0.01 degrees (Check the manual maybe. But seem to be 0.01 for all motors)

        #attempt to connect to the motor via serial
        try:
            self.serial = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout = 0.05)
            self.serial.reset_input_buffer()
        except Exception as e:
            print(f'Exception: {e}')

    def _send_command_with_payload(self, command, payload = b''):
        #send command to motor with payload

        data_len = len(payload)

        #build the header
        header = [0x3E, command, self.MOTOR_ID, data_len]
        header.append(sum(header) % 256) #checksum

        #build the full packet
        full_packet = bytearray(header)
        if data_len > 0:
            full_packet.extend(payload)
            full_packet.append(sum(payload) % 256)

        #send and get response
        self.serial.reset_input_buffer()
        self.serial.write(full_packet)

        #read response
        resp_header = self.serial.read(5)
        if len(resp_header) < 5:
            raise ValueError('Motor responded but response length too short. Check connection with motor.')
        
        #get the rest of the data if needed
        d_len = resp_header[3]
        if d_len > 0:
            payload = self.serial.read(d_len + 1)
            data = payload[:-1]
            d_checksum = payload[-1]

            if sum(data) % 256 != d_checksum:
                raise ValueError('Response checksum failed. Likely poor motor connection')
            return data

        return b''

    def enable_motor(self, command = 0x88):
        return self._send_command_with_payload(command)

    def shutdown_motor(self, command = 0x80):
        return self._send_command_with_payload(command)
    
    def stop_motor(self, command = 0x81):
        return self._send_command_with_payload(command)

    def clear_errors(self, command = 0x9B):
        return self._send_command_with_payload(command)

    def set_speed(self, output_degrees_per_second, command = 0xA2):
        """
        CMD 0xA2: Sets the target speed for the output shaft.
        
        Logic:
        1. Input is degrees/sec at the output shaft.
        2. Multiply by REDUCTION_RATIO to get internal rotor speed.
        3. Multiply by 100 because 1 LSB = 0.01 DPS.
        """
        
        # Convert output shaft speed to internal rotor units
        # Example: 10 deg/s output with i=9 -> 90 deg/s rotor -> 9000 units
        target_speed_units = int(output_degrees_per_second * self.REDUCTION_RATIO * 100)
        
        # Pack into 4 bytes, signed int32, little-endian
        payload_out = struct.pack('<i', target_speed_units)
        
        #send the command to motor
        response_data = self._send_command_with_payload(command, payload_out)

        if not len(response_data) == 7:
            raise ValueError('Incorrect number of bytes received. Check motor connection. Or check manual for expected number of response bytes.')
        
        #parse the response data
        # Temp (1b), IQ (2b), Speed (2b), Encoder (2b)
        temp = struct.unpack('<b', bytes([response_data[0]]))[0]
        iq_raw = int.from_bytes(response_data[1:3], byteorder='little', signed=True)
        speed_raw = int.from_bytes(response_data[3:5], byteorder='little', signed=True)
        encoder_raw = int.from_bytes(response_data[5:7], byteorder='little', signed=False)

        #convert to human readable
        iq_amps = (iq_raw/2048.0) * 33.0 #amps

        #output speed feedback
        output_speed_feedback = speed_raw / self.REDUCTION_RATIO

        return {
            "timestamp": time.time(),
            "temp_c": temp,
            "torque_current_amps": round(iq_amps, 2),
            "output_speed_dps": round(output_speed_feedback, 2),
            "encoder_position": encoder_raw
            }

    def set_position(self, target_angle_deg, max_speed_dps = 100, command = 0xA4):

        """
        max_speed_dps = max angular velocity given in [degrees per second]
        """
        if max_speed_dps < 0:
            raise ValueError('Angular speed must be positive value')

        #The motor accepts angles in units of 0.01 degrees.
        #The motor expects int64_t for the angle
        target_raw = int(target_angle_deg * self.REDUCTION_RATIO * 100)

        #max speed is given as uint32_t
        speed_limit_raw = int(max_speed_dps * self.REDUCTION_RATIO *100)

        #prepare the payload
        #int64_t (8 bytes) + uint32_t (4 bytes)
        payload_out = struct.pack('<qI', target_raw, speed_limit_raw)

        # send and receive response (13 bytes total, 7 bytes data)
        response_data = self._send_command_with_payload(command, payload_out)
        
        if not len(response_data) == 7:
            raise ValueError('Incorrect number of bytes received. Check motor connection. Or check manual for expected number of response bytes.')

        #parse the response data
        # Temp (1b), IQ (2b), Speed (2b), Encoder (2b)
        temp = struct.unpack('<b', bytes([response_data[0]]))[0]
        iq_raw = int.from_bytes(response_data[1:3], byteorder='little', signed=True)
        speed_raw = int.from_bytes(response_data[3:5], byteorder='little', signed=True)
        encoder_raw = int.from_bytes(response_data[5:7], byteorder='little', signed=False)

        #convert to human readable
        iq_amps = (iq_raw/2048.0) * 33.0 #amps

        #output speed feedback
        output_speed_feedback = speed_raw / self.REDUCTION_RATIO

        return {
            "timestamp": time.time(),
            "temp_c": temp,
            "torque_current_amps": round(iq_amps, 2),
            "output_speed_dps": round(output_speed_feedback, 2),
            "encoder_position": encoder_raw
            }


    def get_single_loop_angle(self, command = 0x94):
        # This returns the angle only in the 0 to 360 degree range. 
        # If you need unbounded position of the motor, use get_multi_loop_angle

        #payload = self._query_motor(command=command)
        payload = self._send_command_with_payload(command=command)
        raw_value = int.from_bytes(payload, byteorder='little', signed=True)
        return raw_value * self.ENCODER_RES_DEG/self.REDUCTION_RATIO

    def get_multi_loop_angle(self, command = 0x92):

        # This returns unbounded position of the motor. 
        payload = self._send_command_with_payload(command=command)
        if not len(payload) == 8:
            raise ValueError('Incorrect number of bytes received. Check motor connection. Or check manual for expected number of response bytes.')
        raw_angle = struct.unpack('<q', payload)[0]
        output_shaft_angle = (raw_angle * 0.01) / self.REDUCTION_RATIO
        return round(output_shaft_angle, 3) #resolution is 0.01 degrees.


    def get_motor_status(self, command=0x9A):
        
        #payload = self._query_motor(command=command)
        payload = self._send_command_with_payload(command=command)

        # Temperature at DATA[5] (payload[0])
        temp = struct.unpack('<b', bytes([payload[0]]))[0]
        # Voltage at DATA[7-8] (payload[2-3])
        raw_volts = int.from_bytes(payload[2:4], byteorder='little')
        voltage = raw_volts * 0.1

        # Error byte at DATA[11] (payload[6])
        error_byte = payload[6]

        # Bitwise checks
        low_voltage = bool(error_byte & 0x01)
        over_temp = bool(error_byte & 0x08)

        return {
            "timestamp": time.time(),
            "temp_c": temp,
            "voltage_v": voltage,
            #"error_raw": error_byte,
            "low_volate": low_voltage,
            "over_temp": over_temp,
            "is_healthy": error_byte == 0
        }

    def get_motor_status2(self, command = 0x9C):

        #this one returns more available data by calling two status requests
        multi_loop_angle = self.get_multi_loop_angle()

        #get the current motor status
        status_payload = self._send_command_with_payload(command)
        if not len(status_payload) == 7:
            raise ValueError('Incorrect number of bytes received. Check motor connection. Or check manual for expected number of response bytes.')

        #unpack
        temp = struct.unpack('<b', bytes([status_payload[0]]))[0]
        iq_raw = int.from_bytes(status_payload[1:3], byteorder='little', signed=True)
        speed_raw = int.from_bytes(status_payload[3:5], byteorder='little', signed=True)

        return {
            "timestamp": time.time(),
            "angle_deg": round(multi_loop_angle, 3),
            "velocity_dps": round(speed_raw / self.REDUCTION_RATIO, 2),
            "torque_amps": round((iq_raw / 2048.0) * 33.0, 3),
            "temp_c": temp
        }

if __name__ == '__main__':

    #driver. See initializer for default values
    motor_driver = MotorDriver()

    try:
        motor_status = motor_driver.get_motor_status()
        print('Motor status:', motor_status)

        angle = motor_driver.get_single_loop_angle()
        print(f'Current single rotation angle: {angle: 0.3f} degrees')

        angle = motor_driver.get_multi_loop_angle()
        print(f'current multi loop rotation angle: {angle: 0.3f} degrees')

        motor_status2 = motor_driver.get_motor_status2()
        print(motor_status2)

        if motor_status['is_healthy']:
            resp = input('Do you want to try to rotate the motor? Type [yes]: ')
            if not resp == 'yes': quit()

            #enable to motor
            motor_driver.enable_motor()

            # #spin the motor at a constant rate
            rot_speed = 100.0
            print(f'Spinning the motor at {rot_speed} degrees per second.')
            response = motor_driver.set_speed(rot_speed)
            print(response)

            #rotate the motor for 2 seconds
            time.sleep(2)
            motor_driver.stop_motor()

            # move to a target position
            target_position = -120 #in degrees
            print(f'Moving the motor to postion: {target_position} degrees')
            response = motor_driver.set_position(target_angle_deg=-120, max_speed_dps=50)

            #get feedback while the motor turns.
            while True:
                motor_status2 = motor_driver.get_motor_status2()
                print(motor_status2)
                if abs(motor_status2['velocity_dps']) < 0.001: #break out if the motor stops
                    break
                time.sleep(0.01)        

            #send stop
            motor_driver.stop_motor()

            #servo off?
            #motor_driver.shutdown_motor()

        else:
            resp = input('Motor is not healthy. Try to reset errors? Type [yes]: ')
            if resp == 'yes':
                motor_driver.clear_errors()

    except Exception as e:
        print(f'Exception: {e}')

    finally:
        #always stop the motor when an exception is raised. This should include Control + C terminations.
        motor_driver.stop_motor()