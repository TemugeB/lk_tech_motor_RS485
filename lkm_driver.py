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
            self.serial = serial.Serial(self.SERIAL_PORT, self.BAUDRATE, timeout = 0.1)
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
            raise ValueError('Response length too short. Check connection with motor.')
        
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

        if not response_data or len(response_data) < 7:
            raise ValueError(f'Motor responded with fewer bytes than expected. {response_data}')
        
        #parse the response data
        # Temp (1b), IQ (2b), Speed (2b), Encoder (2b)
        temp = struct.unpack('<b', bytes([response_data[0]]))[0]
        iq_raw = int.from_bytes(response_data[1:3], byteorder='little', signed=True)
        speed_raw = int.from_bytes(response_data[3:5], byteorder='little', signed=True)
        encoder_raw = int.from_bytes(response_data[5:7], byteorder='little', signed=True)

        #convert to human readable
        iq_amps = (iq_raw/2048.0) * 33.0 #amps

        #output speed feedback
        output_speed_feedback = speed_raw / self.REDUCTION_RATIO

        return {
            "temp_c": temp,
            "torque_current_amps": round(iq_amps, 2),
            "output_speed_dps": round(output_speed_feedback, 2),
            "encoder_position": encoder_raw
            }

    def get_current_single_loop_angle(self, command = 0x94):

        #payload = self._query_motor(command=command)
        payload = self._send_command_with_payload(command=command)
        raw_value = int.from_bytes(payload, byteorder='little', signed=True)
        return raw_value * self.ENCODER_RES_DEG/self.REDUCTION_RATIO

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
            "temp_c": temp,
            "voltage_v": voltage,
            #"error_raw": error_byte,
            "low_volate": low_voltage,
            "over_temp": over_temp,
            "is_healthy": error_byte == 0
        }


if __name__ == '__main__':

    motor_driver = MotorDriver()

    try:
        motor_status = motor_driver.get_motor_status()
        print('Motor status:', motor_status)

        angle = motor_driver.get_current_single_loop_angle()
        print(f'Current single rotation angle: {angle: 0.3f}')

        if motor_status['is_healthy']:
            input('Will try to rotate the motor at a constant speed. Press ENTER to continue...')
            
            #enable to motor
            motor_driver.enable_motor()

            #spin the motor
            rot_speed = -150.0
            print(f'Spinning the motor at {rot_speed} degrees per second.')
            response = motor_driver.set_speed(rot_speed)
            print(response)
            time.sleep(10)

            #send stop
            motor_driver.stop_motor()

            #servo off?
            #motor_driver.shutdown_motor()


    except Exception as e:
        print(f'Exception: {e}')