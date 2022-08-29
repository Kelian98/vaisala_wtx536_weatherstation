 #!/usr/env python3
# -*- encoding: utf-8 -*-

# ======================================================================
# LSST-VRO / StarDICE
#
# Low level control for Vaisala WTX536 Weather Station
# See documentation : https://www.vaisala.com/sites/default/files/documents/WXT530-User-Guide-in-English-M211840EN-E.pdf
# ======================================================================

# Authors: K. Sommer, L. Le Guillou
# Email: <kelian.sommer@umontpellier.fr>, <llg@lpnhe.in2p3.fr>
# ======================================================================

import serial
import re

class WTX536(object):
    """
    Class to control the Vaisala WTX536 Weather Station
    """

    # Defined function parameters
    RESET = 'XZ'
    CHECK_SETTINGS = 'XU'
    CHECK_ADDRESS = '?'
    RESET_RAIN_COUNTER = 'XZRU'
    RESET_RAIN_INTENSITY = 'XZRI'
    RESET_MEASUREMENT = 'XZM'
    QUERY_WIND_DATA = 'R1'
    QUERY_AIR_DATA = 'R2'
    QUERY_RAIN_DATA = 'R3'
    QUERY_SUPERVISOR_DATA = 'R5'
    QUERY_ALL_DATA = 'R'
    CHECK_WIND_SETTINGS = 'WU'
    CHECK_AIR_SETTINGS = 'TU'
    CHECK_RAIN_SETTINGS = 'RU'
    CHECK_SUPERVISOR_SETTINGS = 'SU'

    def __init__(self, port = '/dev/ttyUSB0', baudrate=19200, timeout=1, address='0', termination_characters='\r\n', debug=False):
        """
        Parameters
        ----------
        port : str
            Port to connect to the focuser (default = '/dev/ttyUSB0')
        baudrate : int
            Baudrate speed for communication in serial (default = 19200)
        timeout : int
            Timeout for response in seconds (default = 1)
        address : str
            Address of the device
        termination_characters : str
            End of instructions characters
        debug : bool
            If True, print additional information (get and set commands in raw format)
        """
        
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.debug = debug
        self.address = address
        self.termination_characters = termination_characters
        
        if not self.ser.is_open:
            raise Exception("Device not open, check port!")
        else:
            self.check_communication_settings()

    def close(self):
        """Close serial communication with the device"""
        self.ser.close()

    def _write_to_dev(self, cmd, address = True):
        """Low-level method to write to the device

        Parameters
        ----------
        cmd : str
            Command to write to the device
        address : bool
            Whether to set address in the command or not

        Returns
        -------
        retval : str
            Length of cmd sent
        """

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        cmd_to_send = str.encode(self.address+cmd+self.termination_characters)
        if address == False:
            cmd_to_send = str.encode(cmd+self.termination_characters)
        if self.debug == True:
            print('WTX536 WEATHER STATION : SEND BYTES : [' + str(cmd_to_send) + ']')
        retval = self.ser.write(cmd_to_send)
        if retval != len(cmd_to_send):
            raise Exception("wrong returned length %d!=%d. Write to device likely failed"%(retval,len(cmd_to_send)))
        return retval

    def _read_from_dev(self):
        """Low-level method to read from the device

        Returns
        -------
        list           
        """
        
        res = self.ser.read(512)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        try:
            if self.debug == True:
                print('WTX536 WEATHER STATION : RECEIVED BYTES : [' + str(res) + ']')
            print(res.decode('ascii').rstrip().split(sep=','))
            return res.decode('ascii').rstrip().split(sep=',')
        except:
            raise Exception('Parsing read string failed: %s'%res)

    def send(self, cmd, address = True):
        """High-level method to send instruction to the device
        
        Parameters
        ----------
        cmd : str
            Command to write to the device
        address : bool
            Whether to set address in the command or not

        Returns
        -------
        list
        """
        
        self._write_to_dev(cmd, address)
        return self._read_from_dev()

    def reset(self):
        """This command performs software reset on the device."""
        res = self.send(self.RESET)

    def check_communication_settings(self):
        """Use this command to request the current communication settings."""
        res = self.send(self.CHECK_SETTINGS)

    def check_address(self):
        """This command queries the address of the device on the bus."""
        res = self.send(self.CHECK_ADDRESS, False)

    def reset_rain_counter(self):
        """This command resets the rain and hail accumulation and duration parameters Rc, Rd, Hc, and Hd."""
        res = self.send(self.RESET_RAIN_COUNTER)

    def reset_intensity_counter(self):
        """This command resets the rain and hail intensity parameters Ri, Rp, Hi, and Hp."""
        res = self.send(self.RESET_RAIN_INTENSITY)

    def reset_measurement(self):
        """This command interrupts all ongoing measurements except rain measurement and restarts them."""
        res = self.send(self.RESET_MEASUREMENT)

    def echo_test(self):
        """This command checks that a device responds to a data recorder or another device. It asks a sensor to acknowledge its presence on the bus."""
        res = self.send('')
        if res[0] == '0':
            print('WTX536 WEATHER STATION : IS ACTIVE')
        else:
            print('WTX536 WEATHER STATION : CANNOT PING')

    def read_wind_data(self):
        """This command requests the wind data message."""
        res = self.send(self.QUERY_WIND_DATA)
        print('Wind direction minimum [deg] = {}'.format(re.sub('Dn=', '', res[1]).replace('D', '')))
        print('Wind direction average [deg] = {}'.format(re.sub('Dm=', '', res[2]).replace('D', '')))
        print('Wind direction maximum [deg] = {}'.format(re.sub('Dx=', '', res[3]).replace('D', '')))
        print('Wind speed minimum [deg] = {}'.format(re.sub('Sn=', '', res[4]).replace('M', '')))
        print('Wind speed average [deg] = {}'.format(re.sub('Sm=', '', res[5]).replace('M', '')))
        print('Wind speed maximum [deg] = {}'.format(re.sub('Sx=', '', res[6]).replace('M', '')))

    def read_air_data(self):
        """This command requests a pressure, temperature, and humidity data message."""
        res = self.send(self.QUERY_AIR_DATA)
        print('Air temperature [C] = {}'.format(re.sub('Ta=', '', res[1]).replace('C', '')))
        print('Relative humidity [%] = {}'.format(re.sub('Ua=', '', res[2]).replace('P', '')))
        print('Air pressure [Hpa] = {}'.format(re.sub('Pa=', '', res[3]).replace('H', '')))

    def read_rain_data(self):
        """This command requests the precipitation data message."""
        res = self.send(self.QUERY_RAIN_DATA)
        print('Rain accumulation [mm] = {}'.format(re.sub('Rc=', '', res[1]).replace('M', '')))
        print('Rain duration [s] = {}'.format(re.sub('Rd=', '', res[2]).replace('s', '')))
        print('Rain intensity [mm/h] = {}'.format(re.sub('Ri=', '', res[3]).replace('M', '')))
        print('Hail accumulation [hits/cm2] = {}'.format(re.sub('Hc=', '', res[4]).replace('M', '')))
        print('Hail duration [s] = {}'.format(re.sub('Hd=', '', res[5]).replace('s', '')))
        print('Hail intensity [hist/cm2/h] = {}'.format(re.sub('Hi=', '', res[6]).replace('M', '')))

    def read_supervisor_data(self):
        """This command requests a supervisor data message containing self-check parameters of the
heating system and power supply voltage."""
        res = self.send(self.QUERY_SUPERVISOR_DATA)
        print('Heating temperature [C] = {}'.format(re.sub('Th=', '', res[1]).replace('C', '')))
        print('Heating voltage [V] = {}'.format(re.sub('Vh=', '', res[2]).replace('N', '')))
        print('Supply voltage [V] = {}'.format(re.sub('Vs=', '', res[3]).replace('V', '')))
        print('Reference voltage [V] = {}'.format(re.sub('Vr=', '', res[4]).replace('V', '')))

    def read_all_data(self):
        """This command requests all individual messages aR1, aR2, aR3, and aR5 with one command."""
        res = self.send(self.QUERY_ALL_DATA)
        print('Wind direction minimum [deg] = {}'.format(re.sub('Dn=', '', res[1]).replace('D', '')))
        print('Wind direction average [deg] = {}'.format(re.sub('Dm=', '', res[2]).replace('D', '')))
        print('Wind direction maximum [deg] = {}'.format(re.sub('Dx=', '', res[3]).replace('D', '')))
        print('Wind speed minimum [deg] = {}'.format(re.sub('Sn=', '', res[4]).replace('M', '')))
        print('Wind speed average [deg] = {}'.format(re.sub('Sm=', '', res[5]).replace('M', '')))
        print('Wind speed maximum [deg] = {}'.format(re.sub('Sx=', '', res[6]).replace('\r\n0R2', '').replace('M', '')))
        print('Air temperature [C] = {}'.format(re.sub('Ta=', '', res[7]).replace('C', '')))
        print('Relative humidity [%] = {}'.format(re.sub('Ua=', '', res[8]).replace('P', '')))
        print('Air pressure [Hpa] = {}'.format(re.sub('Pa=', '', res[9]).replace('\r\n0R3', '').replace('H', '')))
        print('Rain accumulation [mm] = {}'.format(re.sub('Rc=', '', res[10]).replace('M', '')))
        print('Rain duration [s] = {}'.format(re.sub('Rd=', '', res[11]).replace('s', '')))
        print('Rain intensity [mm/h] = {}'.format(re.sub('Ri=', '', res[12]).replace('M', '')))
        print('Hail accumulation [hits/cm2] = {}'.format(re.sub('Hc=', '', res[13]).replace('M', '')))
        print('Hail duration [s] = {}'.format(re.sub('Hd=', '', res[14]).replace('s', '')))
        print('Hail intensity [hist/cm2/h] = {}'.format(re.sub('Hi=', '', res[15]).replace('\r\n0R5', '').replace('M', '')))
        print('Heating temperature [C] = {}'.format(re.sub('Th=', '', res[16]).replace('C', '')))
        print('Heating voltage [V] = {}'.format(re.sub('Vh=', '', res[17]).replace('N', '')))
        print('Supply voltage [V] = {}'.format(re.sub('Vs=', '', res[18]).replace('V', '')))
        print('Reference voltage [V] = {}'.format(re.sub('Vr=', '', res[19]).replace('V', '')))
        
    def check_wind_settings(self):
        """With the following command you can check the current wind sensor settings."""
        res = self.send(self.CHECK_WIND_SETTINGS)
        print('Wind update interval (1-3600 seconds) = {}'.format(re.sub('I=', '', res[2])))
        print('Wind averaging interval (1-3600 seconds) = {}'.format(re.sub('A=', '', res[3])))
        print('Wind speed calculation mode (1-3) = {}'.format(re.sub('G=', '', res[4])))
        print('Wind speed unit (M = m/s, K = km/h, S = mph, N = knots) = {}'.format(re.sub('U=', '', res[5])))
        print('Wind direction offset (-180 to +180 deg) = {}'.format(re.sub('D=', '', res[6])))
        print('NMEA wind formatter (T = XDR, W = MWV) = {}'.format(re.sub('N=', '', res[7])))
        print('Sampling rate (1 Hz, 2Hz or 4Hz) = {}'.format(re.sub('F=', '', res[8])))

    def check_air_settings(self):
        """Use this command to check the current pressure, temperature, and humidity sensor settings."""
        res = self.send(self.CHECK_AIR_SETTINGS)
        print('Air update interval (1-3600 seconds) = {}'.format(re.sub('I=', '', res[2])))
        print('Pressure unit (H = hPa, P = Pascal, B = bar, M = mmHg, I = inHg) = {}'.format(re.sub('P=', '', res[3])))
        print('Temperature unit (C = Celsius, F = Fahrenheit) = {}'.format(re.sub('T=', '', res[4])))
        print('NMEA air formatter (T = XDR, D = MDA) = {}'.format(re.sub('N=', '', res[5])))
        
    def check_rain_settings(self):
        """Use this command to check the current precipitation sensor settings."""
        res = self.send(self.CHECK_RAIN_SETTINGS)
        print('Rain update interval (1-3600 seconds) = {}'.format(re.sub('I=', '', res[2])))
        print('Rain unit (M = metric, I = imperial) = {}'.format(re.sub('U=', '', res[3])))
        print('Hail unit (M = metric, I = imperial) = {}'.format(re.sub('S=', '', res[4])))
        print('Auto send mode (R = precipitation on/off, C = tipping bucket, T = time based) = {}'.format(re.sub('M=', '', res[5])))
        print('Counter reset mode (M = manual, A = automatic, L= limit, Y = immediate) = {}'.format(re.sub('Z=', '', res[6])))
        print('Rain accumulation limit (100-65535) = {}'.format(re.sub('X=', '', res[7])))
        print('Hail accumulation limit (100-65535) = {}'.format(re.sub('Y=', '', res[8])))

    def check_supervisor_settings(self):
        """Use this command to check the current supervisor settings."""
        res = self.send(self.CHECK_SUPERVISOR_SETTINGS)
        print('Heating update interval (1-3600 seconds) = {}'.format(re.sub('I=', '', res[2])))
        print('Error messaging (Y = enabled, N = disabled) = {}'.format(re.sub('S=', '', res[3])))
        print('Heating control (Y = enabled, N = disabled) = {}'.format(re.sub('H=', '', res[4])))
        
    def set_wind_settings(self, update_time, integration_time, calculation_mode, offset):
        """Modify settings for wind sensor

        Parameters
        ----------
        update_time : int
            Wind sensor update interval (1-3600 seconds)
        integration_time : int
            Wind averaging interval (1-3600 seconds)
        calculation_mode : int
            Wind speed calculation mode (1-3)
        offset : int
            Wind direction offset (-180 to +180 deg)

        """
        res = self.send(self.CHECK_WIND_SETTINGS+',I='+str(update_time)+',A='+str(integration_time)+',G='+str(calculation_mode)+',D='+str(offset))
        print('Wind update interval (1-3600 seconds) = {}'.format(re.sub('I=', '', res[1])))
        print('Wind averaging interval (1-3600 seconds) = {}'.format(re.sub('A=', '', res[2])))
        print('Wind speed calculation mode (1-3) = {}'.format(re.sub('G=', '', res[3])))
        print('Wind direction offset (-180 to +180 deg) = {}'.format(re.sub('D=', '', res[4])))

    def set_air_settings(self, update_time):
        """Modify settings for air sensor

        Parameters
        ----------
        update_time : int
            Air sensor update interval (1-3600 seconds)

        """
        res = self.send(self.CHECK_AIR_SETTINGS+',I='+str(update_time))
        print('Air update interval (1-3600 seconds) = {}'.format(re.sub('I=', '', res[1])))

    def set_rain_settings(self, update_time, counter_reset_mode):
        """Modify settings for rain sensor

        Parameters
        ----------
        update_time : int
            Heating update interval (1-3600 seconds)
        counter_reset_mode : str
            Counter reset mode (M = manual, A = automatic, L= limit, Y = immediate)

        """
        res = self.send(self.CHECK_RAIN_SETTINGS+',I='+str(update_time)+',Z='+str(counter_reset_mode))
        print('Rain update interval (1-3600 seconds) = {}'.format(re.sub('I=', '', res[1])))
        print('Counter reset mode (M = manual, A = automatic, L= limit, Y = immediate) = {}'.format(re.sub('Z=', '', res[2])))

    def set_supervisor_settings(self, update_time, error_mode, heating_mode):
        """Use this command to check the current supervisor settings.
        
        Parameters
        ----------
        update_time : int
            Heating update interval (1-3600 seconds)
        error_mode : str
            Error messaging (Y = enabled, N = disabled)
        heating_mode : str
            Heating control (Y = enabled, N = disabled)

        """
        res = self.send(self.CHECK_SUPERVISOR_SETTINGS+',I='+str(update_time)+',S='+str(error_mode)+',H='+str(heating_mode))
        print('Heating update interval (1-3600 seconds) = {}'.format(re.sub('I=', '', res[1])))
        print('Error messaging (Y = enabled, N = disabled) = {}'.format(re.sub('S=', '', res[2])))
        print('Heating control (Y = enabled, N = disabled) = {}'.format(re.sub('H=', '', res[3])))
