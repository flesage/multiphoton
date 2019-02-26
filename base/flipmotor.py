#!/usr/bin/env python3
# -*- coding:utf-8 -*-
#
# Author: Joel Lefebvre

""" Thorlab's flipper control interface"""

import serial

class flipper():
    def __init__(self, comPort):
        self.transitTime = 500.0; # ms
        self.transitTime_ADC = self.convert_ms2ADC(self.transitTime);
        self.channel = 0x00

        self.destination = 0x50 # General USB Port
        self.source = 0x01 # Host
        self.serial = serial.Serial()
        self.serial.baudrate = 115200
        self.serial.port = comPort
        self.serial.bytesize = serial.EIGHTBITS
        self.serial.stopbits = serial.STOPBITS_ONE
        self.serial.parity = serial.PARITY_NONE
        self.serial.timeout = 10

        #self.state = self.get_state()

    def identify(self):
        """Instruct hardware unit to identify itself (by flashing its front panel LEDs) """
        command = [0x23, 0x02, 0x00, 0x00, self.destination, self.source]
        output = self.write_command(command);
        print output

    def get_hardware_info(self):
        # First send a request for the hardware information
        command = [0x05, 0x00, 0x00, 0x00, self.destination, self.source];
        response = self.write_command(command, responseLength=90)

        # Parse the response
        header = [int(x) for x in response[0:6]]
        serial_number = int.from_bytes(response[6:10], byteorder='little', signed=True)
        model_number = response[10:18].decode("utf-8")
        hardware_type = int.from_bytes(response[18:20], byteorder='little', signed=False)
        firmware_version = [int(x) for x in response[20:24]]
        hardware_version = int.from_bytes(response[84:86], byteorder='little', signed=False)
        mod_state = int.from_bytes(response[86:88], byteorder='little', signed=False)
        nchs = int.from_bytes(response[88::], byteorder='little', signed=False)

        # Print this info
        print("Header: ", header)
        print("Serial Number: ", serial_number)
        print("Model Number: ", model_number)
        print("Hardware Type:", hardware_type)
        print("Firmware Version:", firmware_version)
        print("Hardware Version:", hardware_version)
        print("Mod State:", mod_state)
        print("Number of channels:", nchs)


    def get_MFFstatus(self):
        # Request the MFF operating modes
        command = [0x11, 0x05, self.channel, 0x00, self.destination, self.source]
        response = self.write_command(command, responseLength=40)
        
        
        
        output = dict()
        output['header'] = [hex(x) for x in response[0:6]]
        output['channel'] = int.from_bytes(response[6:8], byteorder='little', signed=False)
        output['transitTime'] = int.from_bytes(response[8:12], byteorder='little', signed=True)
        output['transitTime_ADC'] = int.from_bytes(response[12:16], byteorder='little', signed=True)
        output['operating_mode1'] = int.from_bytes(response[16:18], byteorder='little', signed=False)
        output['operating_mode2'] = int.from_bytes(response[24:26], byteorder='little', signed=False)
        output['signal_mode1'] = int.from_bytes(response[18:20], byteorder='little', signed=False)
        output['signal_mode2'] = int.from_bytes(response[26:28], byteorder='little', signed=False)
        output['pulse_width1'] = int.from_bytes(response[20:24], byteorder='little',signed=True)
        output['pulse_width2'] = int.from_bytes(response[28:32], byteorder='little',signed=True)
        return output

    def get_statusBit(self):
        command = [0x29, 0x04, self.channel, 0x00, self.destination, self.source]
        response = self.write_command(command, responseLength=12)
        output = dict()
        output['header'] = [hex(x) for x in response[0:6]]
        output['channel'] = int.from_bytes(response[6:8], byteorder='little', signed=False)
        output['status'] = int.from_bytes(response[8:12], byteorder='little', signed=False)
        return output

    def get_motorStatus(self):
        command = [0x80, 0x04, self.channel, 0x00, self.destination, self.source]
        response = self.write_command(command, responseLength=20)
        
        print repr(response[16:19])
        print int(response[16:19].encode('hex'),16)
        print repr(response)
        
        output = dict()
        output['header'] = [x for x in response[0:6]]
        output['channel'] = int(response[6:8].encode('hex'),16)
        output['position'] = int(response[8:12].encode('hex'),16)
        output['velocity'] = int(response[12:14].encode('hex'),16)
        #output['status_bit'] = int.from_bytes(response[16:20], byteorder='little', signed=False)
        output['status_bit'] = int(response[16:19].encode('hex'),16)
        
        print output
        
        if output['status_bit'] == 196608:
            output['state'] = 'up'
        else:
            output['state'] = 'down'
        
        print output
        
        return output

    def get_state(self):
        response = self.get_motorStatus()
        return response['state']

    def switch(self):
        # Get the current state
        response = self.get_motorStatus();
        if response['state'] == 'up':
            direction = 0x02
        else:
            direction = 0x01

        command = [0x6A, 0x04, self.channel, direction, self.destination, self.source]
        self.write_command(command, responseLength=0)












    def move_up(self):
        command = [0x6A, 0x04, self.channel, 0x01, self.destination, self.source]
        self.write_command(command, responseLength=0)

    def move_down(self):
        command = [0x6A, 0x04, self.channel, 0x02, self.destination, self.source]
        self.write_command(command)












    def move(self, pos, inverse=False):
        """ Move to the 'up' or the 'down' position """
        assert pos in ['up', 'down'], "Positions must be either 'up' or 'down'"
        if pos == 'up':
            if inverse:
                self.move_down()
            else:
                self.move_up()
        elif pos == 'down':
            if inverse:
                self.move_up()
            else:
                self.move_down()

    def write_command(self, command, convertHex=True, responseLength=None):
        assert isinstance(command, list), "'command' must be a list"

        if convertHex:
            for ii, x in enumerate(command):
                if isinstance(x, str):
                    command[ii] = int(x,16)
                else:
                    command[ii] = int(x)

        # Write a command
        
        self.serial.open()
        self.serial.flushInput()
        self.serial.flushOutput()
        self.serial.write(command)

        # Read the command response (if any)
        if responseLength is None:
            response = self.serial.readline()
        elif responseLength > 0:
            response = self.serial.read(responseLength)
        else:
            response = None
        self.serial.close()

        return response
        #return [x for x in response]

    def print_status(self):
        pass

    def convert_ms2ADC(self, time_ms):
        time_adc = int(1e7 * time_ms ** (-1.591))
        return time_adc

        # def waitForMoveToEnd(self):
        #     still_moving = True
        #     while still_moving:
        #         time.sleep(self.update_interval)
        #         status = self.get_status()
        #         still_moving = status['isMoving']
        #     time.sleep(self.update_interval)


    def __del__(self):
        self.serial.close()