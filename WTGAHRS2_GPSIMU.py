import serial
import struct

class WTGAHRS2:
    def __init__(self, port='COM6', baudrate=9600):
        self.packets = {}
        self.ser = serial.Serial(port, baudrate, timeout=1)

    def get_packets(self):
        return self.packets

    def get_time(self):
        return self.packets.get('time')

    def get_acceleration(self):
        return self.packets.get('acceleration')

    def get_angular_velocity(self):
        return self.packets.get('angular_velocity')

    def get_angles(self):
        return self.packets.get('angles')

    def get_magnetic_field(self):
        return self.packets.get('magnetic_field')

    def get_pressure_altitude(self):
        return self.packets.get('pressure_altitude')

    def get_lat_lon(self):
        return self.packets.get('lat_lon')

    def get_height_yaw_velocity(self):
        return self.packets.get('height_yaw_velocity')

    @staticmethod
    def parse_data_packet(buffer):
        index = 0
        packets = {}

        while index + 1 < len(buffer):
            if buffer[index] == 0x55:  # Header byte
                type_byte = buffer[index + 1]
                packet_length = 14  # Standard packet length for most types

                if index + packet_length <= len(buffer):
                    packet_data = buffer[index:index + packet_length]
                    index += packet_length

                    # Process each type of packet
                    type_handlers = {
                        0x50: ('time', '<BBBBBB'), # year, month, day, hour, minute, second
                        0x51: ('acceleration', '<hhh', lambda x: [i * 16 / 32768 for i in x]), # m/s²
                        0x52: ('angular_velocity', '<hhh', lambda x: [i * 2000 / 32768 for i in x]), # °/s
                        0x53: ('angles', '<hhh', lambda x: [i * 180 / 32768 for i in x]), # degrees
                        0x54: ('magnetic_field', '<hhh', lambda x: [i / 117 for i in x]), # µT
                        0x56: ('pressure_altitude', '<ff', lambda x: [i / 0.01401e-43 for i in x]), # Pa, cm
                        0x57: ('lon_lat', '<ii', lambda x: [i / 10000000 for i in x]), # degrees
                        0x58: ('height_yaw_velocity', '<hhf')
                    }
                    handler = type_handlers.get(type_byte)
                    if handler:
                        format_string = handler[1]
                        func = handler[2] if len(handler) > 2 else None
                        unpacked_data = struct.unpack(format_string, packet_data[2:2 + struct.calcsize(format_string)])
                        packets[handler[0]] = func(unpacked_data) if func else unpacked_data
                else:
                    break
            else:
                index += 1
        return packets, buffer[index:]

    def read_and_parse_data(self):
        buffer = bytes()
        while True:
            data = self.ser.read(self.ser.in_waiting or 1)
            if not data:
                continue

            buffer += data
            packets, buffer = self.parse_data_packet(buffer)
            self.print_packets(packets)

    @staticmethod
    def print_packets(packets):
        for key, value in packets.items():
            print(f"{key}: {value}")

    def update(self, buffer=bytes()):
        data = self.ser.read(self.ser.in_waiting or 1)
        if not data:
            return
        buffer += data
        packets, buffer = self.parse_data_packet(buffer)
        self.print_packets(packets)
        

def main():
    wtg = WTGAHRS2('COM6', 9600)
    buffer = bytes()
    while True:
        wtg.update(buffer)

if __name__ == '__main__':
    main()
