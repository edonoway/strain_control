'''
Starts strain control server application, which coordiantes Keysight LCR meter and Razrbill RP100 power supply to control for strain on the Razorbill CSX130 strain cell.

The server operates over various threads: (1) a main GUI thread for displaying and interacting with instruments. (2) a control loop which include options for setting power supply output voltage to achieve a desired strain and PID control. (3) a monitoring loop for reading and logging instrument values. (4) a communications loop for recieving and responding to commands from the strain client via python's implementation of socket programming.

SOME IMPORTANT SAFETY NOTES:
- include proper voltage limits for the power supply (ideally as a function of temperature with some backups safety to ensure it defaults to lowest limits)
- wire both the power supply and the capacitor correctly by rereading appropriate sections in the manual

To Do:
(1) add logging
(2) read temperature on lakeshore

As time allows:
(3) Change global variables to configuration file.
(4) fix PID and how it interacts with rough ramp
(5) fix set_strain() - add some feedback control to rough ramp, or an option to do so

'''

from concurrency_classes import LockedVar, StoppableThread, queue_write, queue_read
from simulation import SimulatedLCR, SimulatedPS, SimulatedMontana
from pymeasure.instruments.agilent import AgilentE4980
from pymeasure.instruments.razorbill import razorbillRP100
import pyvisa
import OrensteinLab_git.Instrument.montana.cryocore as cryocore
from simple_pid import PID
from threading import Thread, Lock, Event
import threading
import multiprocessing as mp
from multiprocessing import Queue
import matplotlib.pyplot as plt
import numpy as np
import datetime
import time
import sys
import socket
import re
import pyqtgraph as pg
from pyqtgraph import QtCore, QtWidgets

##########################
### USER SETTINGS HERE ###
##########################
global SIM, STARTING_SETPOINT, SLEW_RATE, P, I, D, L0, MAX_VOLTAGE, MIN_VOLTAGE, HOST, PORT, LCR_ADDRESS, PS_ADDRESS

SIM=True
STARTING_SETPOINT=0
SLEW_RATE=0.5
P=100
I=100
D=0.1
L0 = 68.68 # initial capacitor spacing
L0_SAMP = 68.68
C_MEASURED_0 = 0.812 # pF, measured capacitance at 300K and 0V after a zeroing procedure. (Used to be 0.824)
C_0 = 0.808 # pF, true capacitance at 300K and 0 V.

### LIMIT OUTPUT VOLTAGE HERE ###

MAX_VOLTAGE = 5#119 # V
MIN_VOLTAGE = -5#-19 # V

### COMMUNICATION SETTINGS ###
LCR_ADDRESS = 'USB0::0x2A8D::0x2F01::MY54412905::0::INSTR'
PS_ADDRESS = 'ASRL4::INSTR'
MONTANA_ADDRESS = '10.1.1.15'
LAKESHORE_ADDRESS = ''
HOST = 'localhost'
PORT = 15200

###########################
###########################
###########################

class StrainServer:

    def __init__(self, lcr, ps, cryo, serversocket, setpoint, p, i, d, l0_samp, l0=68.68, logging_interval=1., sim=False):
        '''
        class constructor.

        args:
            - lcr                      pymeasure LCR handle or simulation object
            - ps:                      pymeasure power supply handle or simulation object
            - cryo:                    Montana CryoCore object
            - s:                       a bound socket for communicating with strain client.
            - setpoint(float):         initial setpoint for PID
            - P(float):                proportional PID parameter
            - I(float):                integral PID parameter
            - D(float):                derivative PID parameter
            - l0_samp(float):          initial length of sample
            - l0(float):               initial capacitor gap
            - logging_interval(float): time interval between data captures for file log

        returns: class instance object
        '''
        self.lcr = lcr
        self.ps = ps
        self.cryo = cryo
        self.serversocket = s
        self.l0 = l0
        self.l0_samp = LockedVar(l0_samp)
        self.logging_interval = LockedVar(logging_interval)
        self.sim = LockedVar(sim)
        temperature = self.cryo.get_platform_temperature()[1]
        self.temperature = LockedVar(temperature)
        strain, cap, imaginary_impedance, dl = self.get_strain()
        self.strain = LockedVar(strain)
        self.setpoint = LockedVar(setpoint)
        self.cap = LockedVar(cap)
        self.imaginary_impedance = LockedVar(imaginary_impedance)
        self.dl = LockedVar(dl)
        v1, v2 = self.get_voltage(1), self.get_voltage(2)
        self.voltage_1 = LockedVar(v1)
        self.voltage_2 = LockedVar(v2)
        self.output_1 = LockedVar(False)
        self.output_2 = LockedVar(False)
        self.max_voltage_1 = LockedVar(MAX_VOLTAGE)
        self.min_voltage_1 = LockedVar(MIN_VOLTAGE)
        self.max_voltage_2 = LockedVar(MAX_VOLTAGE)
        self.min_voltage_2 = LockedVar(MIN_VOLTAGE)
        self.slew_rate = LockedVar(SLEW_RATE)
        self.pid = PID(p, i, d, setpoint=self.setpoint.locked_read())
        #self.pid.sample_time = 0.01
        self.p, self.i, self.d = [LockedVar(j) for j in self.pid.tunings]
        self.ctrl_mode = LockedVar(1)
        self.ctrl_status = LockedVar(0)
        self.run = LockedVar(True)
        self.host = HOST
        self.port = PORT
        # filename_head = r'C:\Users\orens\Google Drive\Shared drives\Orenstein Lab\Data\Strain cell log files'
        # filename = '\\' + time.strftime("%y%m%d_%H:%M:%S%z", time.localtime()) + 'StrainServerLog.dat'
        # tot_filename = filename_head + filename
        # self.filepath = LockedVar(tot_filename)

    def initialize_instruments(self):
        '''
        Sets initial setting and paramters for both LCR meter and power supply.
        '''
        self.set_slew_rate(SLEW_RATE)
        self.set_output(1,0)
        self.set_output(2,0)
        if self.sim.locked_read()==False:
            self.lcr.mode = "CPD"
            self.lcr.frequency = 3e5 # 300kHz
        return 1

    def start_strain_control(self, mode):
        '''
        High level handling of strain control. For now, sets new strain value by first slowly ramping voltage to approximate voltage and then maintaining strain with a restricted PID loop.

        args:
            - mode(int):     1:'PID, 2:'Set Voltage', or 3:'Combined'

        returns: None
        '''

        current_setpoint = self.setpoint.locked_read()
        self.ctrl_status.locked_update(1)
        queue_write(self.ctrl_status_q, 1)
        if mode==1:
            pid_loop = StoppableThread(target=self.start_pid, args=(current_setpoint,), kwargs={'limit':False})
            print('Starting PID control')
            pid_loop.start()
            current_thread = threading.current_thread()
            while current_thread.stopped()==False:
                new_setpoint = self.setpoint.locked_read()
                current_setpoint = new_setpoint
                self.pid.setpoint = current_setpoint
            print('Stopping PID control')
            pid_loop.stop()
            pid_loop.join()

        elif mode==2:

            print('Starting constant voltage control')
            self.set_strain(current_setpoint)

            current_thread = threading.current_thread()
            while current_thread.stopped()==False:
                new_setpoint = self.setpoint.locked_read()
                if new_setpoint != current_setpoint:
                    current_setpoint = new_setpoint
                    self.set_strain(current_setpoint)
            print('Stopping constant voltage control')

        elif mode==3:

            pid_loop = StoppableThread(target=self.start_pid, args=(current_setpoint,), kwargs={'limit':False})
            print('Setting strain with fixed voltage')
            self.set_strain(current_setpoint)
            print('Strain achieved, starting PID control')
            pid_loop.start()

            current_thread = threading.current_thread()
            while current_thread.stopped()==False:
                new_setpoint = self.setpoint.locked_read()
                if new_setpoint != current_setpoint:
                    current_setpoint = new_setpoint
                    if pid_loop.is_alive():
                        pid_loop.stop()
                        pid_loop.join()
                        print('Stopping PID control')
                    pid_loop = StoppableThread(target=self.start_pid, args=(current_setpoint,), kwargs={'limit':False})
                    print('Setting strain with fixed voltage')
                    self.set_strain(current_setpoint)
                    print('Strain achieved, starting PID control')
                    pid_loop.start()

            print('Stopping PID control')
            pid_loop.stop()
            pid_loop.join()

        self.ctrl_status.locked_update(0)
        queue_write(self.ctrl_status_q, 0)
        print('Shut down control thread')

    def start_strain_monitor(self):
        '''
        Continuously reads lcr meter and ps and updates all state variables to class instance variables. In addition, in the future this should handle logging of instrument data.
        '''
        print('Starting strain monitor')
        current_thread = threading.current_thread()
        while current_thread.stopped() == False:
            strain, cap, imaginary_impedance, dl = self.get_strain()
            temperature = self.cryo.get_platform_temperature()[1]
            v1 = self.get_voltage(1)
            v2 = self.get_voltage(2)
            out1 = self.get_output(1)
            out2 = self.get_output(2)
            self.temperature.locked_update(temperature)
            self.strain.locked_update(strain)
            self.cap.locked_update(cap)
            self.imaginary_impedance.locked_update(imaginary_impedance)
            self.dl.locked_update(dl)
            self.voltage_1.locked_update(v1)
            self.voltage_2.locked_update(v2)
            self.output_1.locked_update(out1)
            self.output_2.locked_update(out2)
            # update queues
            queue_update = [self.strain_q, self.cap_q, self.dl_q, self.voltage_1_q, self.voltage_2_q, self.output_1_q, self.output_2_q, self.temperature_q]
            state_values = [strain, cap, dl, v1, v2, out1, out2, temperature]
            for ii, q in enumerate(queue_update):
                queue_write(q, state_values[ii])
            time.sleep(0.1)
        print('Shut down monitor thread')

    def start_comms(self):
        '''
        start listening to serversocket and respond to connect requests with typical socket communications.
        '''
        print(f'Opening communication socket on {self.host} at port {self.port}')
        current_thread = threading.current_thread()
        while True:
            if current_thread.stopped()==False:
                self.serversocket.listen(1)
                print('Listening for strain client')
                conn, addr = self.serversocket.accept()
                with conn:
                    #print(f'Connected to strain client at address {addr}')
                    while True: # run main loop
                        message = conn.recv(1024)
                        #print('Receive from client initiated.')
                        if not message:
                            #print('Message received and strain client terminated connection.')
                            break
                        decoded_message = message.decode('utf8')
                        try:
                            response = self.parse_message(decoded_message)
                        except:
                            error_msg = 'Error: unable to parse message: '+str(message)
                            print(error_msg)
                            conn.sendall(error_msg.encode('utf8'))
                            break
                        try:
                            #print('Transmitting response to client')
                            conn.sendall(response.encode('utf8'))
                        except:
                            error_msg = 'Error: unable to transmit response to client.'
                            print(error_msg)
                            conn.sendall(error_msg.encode('utf8'))
                            break
            else:
                break
        print('Shut down communications thread')

    def filelog(self, time_interval):
        '''
        Logs all data in main loop to log file.

        args:
            - time_interval(float): time interval in seconds between measuring device state information for writing to log file.

        returns: None
        '''
        print('Starting file log')
        filename_head = r'C:\Users\orens\Google Drive\Shared drives\Orenstein Lab\Data\Strain cell log files'
        filename = f'\StrainServerLog_{time.strftime("%y%m%d_%H.%M.%S%z", time.localtime())}.dat'
        tot_filename = filename_head + filename
        self.filepath = LockedVar(tot_filename)
        time_interval = self.logging_interval.locked_read()
        current_thread = threading.current_thread()
        with open(self.filepath.locked_read(), 'a') as f:
            f.write(f'Time\tStrain\tPID Setpoint\tCapacitance (pF)\tdl (um)\tSample Length(um)\tVoltage 1 (V)\tVoltage 2 (V)\tOutput 1\tOutput 2\tP\tI\tD\tMin Voltage 1\tMin Voltage 2\tMax Voltage 1\tMax Voltage 2\tSlew Rate\tMode\tStatus\tRun\tTemperature (K)\n')
        while current_thread.stopped() == False:
            queues = [self.strain_q, self.setpoint_q, self.cap_q, self.dl_q, self.l0_samp_q, self.voltage_1_q, self.voltage_2_q, self.output_1_q, self.output_2_q, self.p_q, self.i_q, self.d_q, self.min_voltage_1_q, self.min_voltage_2_q, self.max_voltage_1_q, self.max_voltage_2_q, self.slew_rate_q, self.ctrl_mode_q, self.ctrl_status_q, self.run_q, self.temperature_q]
            with open(self.filepath.locked_read(), 'a') as f:
                f.write(str(datetime.datetime.now())+'\t')
                allbutlast = ''.join(str(queue_read(logval))+'\t' for logval in queues[:-1])
                f.write(allbutlast)
                f.write(str(queue_read(queues[-1]))+'\n')
            time.sleep(time_interval)
        print('Shutting down file log thread')

    def set_strain(self, setpoint):
        '''
        Ramp voltage on power supply to an approximately correct strain, returning once that strain has been achieved within tolerance. This can be proceeded by PID control.

        args:
            - setpoint(float):  strain setpoint. We take this as an explicit parameter to avoid possible conflicts and make this function cleaner.

        returns: None

        '''
        setpoint_val = self.setpoint.locked_read()
        strain_val = self.strain.locked_read()
        start_voltage = self.strain_to_voltage(setpoint_val)
        voltage_increment = 0.5
        strain_tol=0.005
        direction = (setpoint_val - strain_val)/abs(setpoint_val - strain_val)
        n=0
        while abs(strain_val) <= abs(setpoint_val):
            approx_voltage = start_voltage + n*voltage_increment
            self.ps_write(approx_voltage)

            loop_cond = True
            while loop_cond:
                v1, v2 = self.get_voltage(1), self.get_voltage(2)
                if v1 > (approx_voltage - self.ps.tol) or v1 < (approx_voltage + self.ps.tol):
                    loop_cond = False
                strain_val = self.strain.locked_read()
                if abs(strain_val) >= abs(setpoint_val):
                    loop_cond = False
            n=n+1

    def start_pid(self, setpoint, limit=False):
        '''
        Start PID loop to control strain.

        args:
            - setpoint(float):      PID setpoint. We take this as an explicit parameter to avoid possible conflicts and make this function cleaner.

        returns: None

        kwargs:
            - limit(bool):
        '''
        if limit==True:
            v1, v2 = self.ps_read()
            v0 = v1

        self.output_limits = (MIN_VOLTAGE, MAX_VOLTAGE)
        self.pid.setpoint = setpoint
        current_thread = threading.current_thread()
        while current_thread.stopped()==False:
            # compute new output given current strain
            new_voltage = self.pid(self.strain.locked_read())
            if limit==True:
                dv = new_voltage - v0
                if abs(dv) > 5:
                    new_voltage = v0 + 5*(dv/abs(dv))
            #print(new_voltage)
            # set the new output and get current value
            self.ps_write(new_voltage)
            time.sleep(0.01)

    def ps_write(self, voltage):
        '''
        update both channels of power supply to new voltage. change in future to coordinate the voltages in the best way (ie, not just same voltage on each channel, maybe we just pick both?) - really, I think this function may eventually use some other data such as direction of applied voltage or setpoint-strain to determine which channel should be energized corresponding to compression or tension.
        '''
        self.set_voltage(1, voltage)
        #self.set_voltage(2, voltage)

    def set_voltage(self, channel, voltage):
        '''
        update power supply voltage within proper limits.

        args:
            - channel(int):         channel on ps to set, must be 1 or 2
            - voltage(float):       voltage to set on power supply.

        returns: None

        '''
        try:
            if not (channel==1 or channel==2):
                raise ValueError('channel must be int 1 or 2.')
            # limit max/min voltage
            if channel==1:
                max = self.max_voltage_1.locked_read()
                min = self.min_voltage_1.locked_read()
                if voltage > max:
                    voltage = max
                elif voltage < min:
                    voltage = min
            elif channel==2:
                max = self.max_voltage_2.locked_read()
                min = self.min_voltage_2.locked_read()
                if voltage > max:
                    voltage = max
                elif voltage < min:
                    voltage = min
            # set voltages
            if self.sim.locked_read()==True:
                self.ps.set_voltage(channel, voltage)
            else:
                if channel==1:
                    self.ps.voltage_1 = voltage
                elif channel==2:
                    self.ps.voltage_2 = voltage
        #print(f'Ramping voltage on channel {channel} to {voltage} V')
        except:
            print('Error: unable to set voltage.')

    def get_voltage(self, channel):
        '''
        returns voltage 1 or voltage 2 from power supply.

        args:
            - channel(int)

        returns:
            - v1(float)
            - v2(float)s

        '''
        try:
            if not (channel==1 or channel==2):
                raise ValueError('channel must be int 1 or 2.')
            if self.sim.locked_read()==True:
                if channel==1:
                    v = self.ps.voltage_1.locked_read()
                elif channel==2:
                    v = self.ps.voltage_2.locked_read()
            else:
                if channel==1:
                    v = self.ps.instant_voltage_1
                elif channel==2:
                    v = self.ps.instant_voltage_2
            return v
        except:
            print('Error: unable to read voltage.')

    def set_output(self, channel, state):
        '''
        sets the output status of power supply on channel.

        args:
            - channel(int)
            - state(int):           0 or 1

        return: None
        '''
        try:
            if not (channel==1 or channel==2):
                raise ValueError('channel must be int 1 or 2.')
            if not (state==0 or state==1):
                raise ValueError('output state must be 1 or 0')
            if self.sim.locked_read()==True:
                if channel==1:
                    v = self.ps.output_1.locked_update(state)
                elif channel==2:
                    v = self.ps.output_2.locked_update(state)
            else:
                if channel==1:
                    self.ps.output_1 = state
                elif channel==2:
                    self.ps.output_2 = state
        except:
            print('Error: unable to set output.')

    def get_output(self, channel):
        '''
        returns the status of power supply on channel

        args: None

        returns:
            - state(int)
        '''
        try:
            if not (channel==1 or channel==2):
                raise ValueError('channel must be int 1 or 2.')
            if self.sim.locked_read()==True:
                if channel==1:
                    state = self.ps.output_1.locked_read()
                elif channel==2:
                    state = self.ps.output_2.locked_read()
            else:
                if channel==1:
                    state = self.ps.output_1
                elif channel==2:
                    state = self.ps.output_2
            return state
        except:
            print('Error: unable to read output.')

    def set_slew_rate(self, slew_rate):
        '''
        utility to set slew rate on power supply on both channels
        '''
        # unpack queues
        print(f'Setting slew rate on power supply to {slew_rate} V/s')
        if self.sim.locked_read()==True:
            self.ps.slew_rate.locked_update(slew_rate)
        else:
            self.ps.slew_rate_1 = slew_rate
            self.ps.slew_rate_2 = slew_rate
        self.slew_rate.locked_update(slew_rate)
        queue_write(self.slew_rate_q, slew_rate)

    def get_strain(self):
        '''
        Querys LCR meter for current impedance measurement and uses calibration curve to return strain. Assumes that LCR measurement mode is set to one of the parallel modes, which is appropriate for measuring small capacitance (such as CPD)

        args: None

        returs:
            - strain(float):        calculated strain
            - l(float):             gap between sample plates in um
            - dl(float):            l - l0
        '''
        impedance = self.lcr.impedance # or read impedance as posted by another process
        cap = impedance[0]*1e12 # capacitance in pF
        imaginary_impedance = impedance[1]
        dl = self.capacitance_to_dl(cap)
        strain = dl/self.l0_samp.locked_read()
        return strain, cap, imaginary_impedance, dl

    def capacitance_to_dl(self, capacitance_measured):
        '''
        helper function that returns change in gap between sample plates from initial gap (dl = l - l0) given a capacitance reading based on the CS130 capacitor calibration.

        The capacitance can be thought of as comprising four parts:

        C_measured(V, T) = deltaC(V) + C_0 + C_parasitic + C_temp(T),

        where,

        C_true = deltaC(V) + C_0 = C_measured(V,T) - C_parasitic - C_temp(T)

        The "true" capacitance we want is the 0 strain capacitance C_0 plus the voltage induced change deltaC(V). The parasitic capacitance can be obtained by doing a proper "zeroing" procedure at room temperature and subtracting off the known "true" value at 0 volts, ie since we know C_0 = 0.808 pF,

        C_parasitic = C_measured(0,300) - 0.808 pF.

        The temperature induced offset should also be calculated as

        C_temp(T) = C_measured(0,T) - C_measured(0,300)

        args:
            - capacitance(float):         capacitance in pF

        returns:
            - dl(float):                  l - l0, the change in gap between sample plates from initial value in um
        '''
        # capacitor specifications
        area = 5.95e6 # um^2
        eps0 = 8.854e-6 # pF/um - vacuum permitivity
        capacitance_parasitic = C_MEASURED_0 - C_0
        capacitance_temp = self.capacitance_temperature_offset(self.temperature.locked_read())
        capacitance_true = capacitance_measured - capacitance_parasitic - capacitance_temp
        l0 = self.l0 # um
        cap_offset = 0.04 # pf - this appears in the equation for finding dl from the true capactiance.
        dl = eps0*area/(capacitance_true - cap_offset) - l0 # um
        return dl

    def capacitance_temperature_offset(self, temperature):
        '''
        Calculate the temperature induced offset to capacitance in pF.

        args:
            - temperature(float):       temperature in K

        returns:
            - offset(float):            capacitance offset, ie
                                        C_measured(0,temp) - C_measured(0,300)
        '''

        c_measured0 = C_MEASURED_0
        c_measuredT = c_measured0 # change to reflect to calibartion curve. may need to find a good way to interpolate data

        return c_measuredT - c_measured0

    def strain_to_voltage(self, strain):
        '''
        Helper function that returns power supply voltage estimated for a desired strain.

        args:
            - strain(float):        desired strain

        returns:
            - voltage(float):       estimated required voltage to achieve strain
        '''
        l0 = self.l0_samps
        response = 0.05 # um/V
        dl = strain*l0
        voltage = dl/response
        return voltage

    def parse_message(self, message):
        '''
        Utility that implements communications protocol with strain client. If message is not parable, error handling is handled by communication loop.

        args:
            - message(string):

        returns:
            - response(string):
        '''
        if re.match(r'SCTRL:[1-3]', message):
            mode = int(re.search(r'[1-3]', message)[0])
            if self.strain_control_loop.is_alive():
                current_mode = self.ctrl_mode.locked_read()
                if mode!=current_mode:
                    print(f'Stopping control thread in mode {current_mode} and restarting in mode {mode}')
                    self.ctrl_mode.locked_update(mode)
                    queue_write(self.ctrl_mode_q, mode)
                    self.strain_control_loop.stop()
                    self.strain_control_loop.join()
                    self.strain_control_loop = StoppableThread(target=self.start_strain_control, args=(mode,))
                    self.strain_control_loop.start()
                else:
                    print(f'Control thread in mode {mode} already in progress, no action taken')
            else:
                print(f'Starting control thread in mode {mode}')
                self.ctrl_mode.locked_update(mode)
                queue_write(self.ctrl_mode_q, mode)
                self.strain_control_loop = StoppableThread(target=self.start_strain_control, args=(mode,))
                self.strain_control_loop.start()
            response = '1'
        elif message == 'ECTRL:':
            if self.strain_control_loop.is_alive():
                v1, v2 = self.get_voltage(1), self.get_voltage(2)
                self.strain_control_loop.stop()
                self.strain_control_loop.join()
                self.set_voltage(1,v1)
                self.set_voltage(1,v2)
            response = '1'
        elif message == 'STR:?':
            response = str(self.strain.locked_read())
        elif message == 'DL:?':
            response = str(self.dl.locked_read())
        elif message == 'CAP:?':
            response = str(self.cap.locked_read())
        elif re.match(r'STR:-?[0-9]+[\.]?[0-9]*', message):
            setpoint = float(re.search(r'-?[0-9]+[\.]?[0-9]*', message)[0])
            self.setpoint.locked_update(setpoint)
            queue_write(self.setpoint_q, setpoint)
            response = '1'
        elif re.match(r'VOL[1-2]:\?', message):
            channel = int(re.search(r'[1-2]', message)[0])
            v = self.get_voltage(channel)
            response = str(v)
        elif re.match(r'OUT[1-2]:[0-1]', message):
            channel = int(re.search(r'[1-2]', message)[0])
            state = int(re.search(r':[0-1]', message)[0][1])
            self.set_output(channel, state)
            response = '1'
        elif re.match(r'OUT[1-2]:?', message):
            channel = int(re.search(r'[1-2]', message)[0])
            state = self.get_output(channel)
            response = str(state)
        elif re.match(r'VOL[1-2]:-?[0-9]+[\.]?[0-9]*', message):
            channel = int(re.search(r'[1-2]', message)[0])
            voltage = float(re.findall(r'-?[0-9]+[\.]?[0-9]*', message)[1])
            self.set_voltage(channel, voltage) # change ps_write to specify channel as well.
            response = '1'
        elif re.match(r'VLIMS[1-2]:-?[0-9]+[\.]?[0-9]*,-?[0-9]+[\.]?[0-9]*',message):
            channel = int(re.search(r'[1-2]', message)[0])
            min, max = [float(i) for i in re.findall(r'-?[0-9]+[\.]?[0-9]*', message)[1:]]
            if channel==1:
                self.min_voltage_1.locked_update(min)
                self.max_voltage_1.locked_update(max)
                queue_write(self.min_voltage_1_q, min)
                queue_write(self.max_voltage_1_q, max)
                response = '1'
            elif channel==2:
                self.min_voltage_2.locked_update(min)
                self.max_voltage_2.locked_update(max)
                queue_write(self.min_voltage_2_q, min)
                queue_write(self.max_voltage_2_q, max)
                response = '1'
            else:
                response = 'Invalid channel'
        elif re.match(r'SAMPL0:[0-9]+[\.]?[0-9]*', message):
            samp_l0 = float(re.findall(r'[0-9]+[\.]?[0-9]*', message)[1])
            self.l0_samp.locked_update(samp_l0)
            queue_write(self.l0_samp_q, samp_l0)
            response='1'
        elif re.match(r'PID:[0-9]+[\.]?[0-9]*,[0-9]+[\.]?[0-9]*,[0-9]+[\.]?[0-9]*', message):
            p, i, d = [float(j) for j in re.findall(r'[0-9]+[\.]?[0-9]*', message)]
            self.pid.tunings = (p,i,d)
            self.p.locked_update(p)
            self.i.locked_update(i)
            self.d.locked_update(d)
            queue_write(self.p_q, p)
            queue_write(self.i_q, i)
            queue_write(self.d_q, d)
            response = '1'
        elif re.match(r'VSLW:[0-9]+[\.]?[0-9]*', message):
            slew_rate = float(re.search(r'[0-9]+[\.]?[0-9]*', message)[0])
            self.set_slew_rate(slew_rate)
            response = '1'
        elif re.match(r'SHTDWN:[0-1]', message):
            mode = int(re.search(r'[0-1]', message)[0])
            self.shutdown(mode)
            response = '1'
        return response

    def shutdown(self, mode):
        '''
        Initiates shutdown of server.

        args:
            - mode(int):        0 to leave state of system as is, or 1 to ramp voltages down to 0.
            - run_q

        returns: None
        '''
        print('Shutting down strain server:')
        if self.comms_loop.is_alive():
            self.comms_loop.stop()
            # can't join because we might be in it!
        if self.strain_control_loop.is_alive():
            self.strain_control_loop.stop()
            self.strain_control_loop.join()
        if mode==1:
            print('Ramping voltage on all channels to 0')
            self.set_voltage(1, 0)
            self.set_voltage(2, 0)
            eps = 0.1
            while (np.abs(self.get_voltage(1)) >= eps) or (np.abs(self.get_voltage(1)) >= eps):
                continue
            self.set_output(1,0)
            self.set_output(2,0)
        if self.strain_monitor_loop.is_alive():
            self.strain_monitor_loop.stop()
            self.strain_monitor_loop.join()
        self.run.locked_update(False)
        queue_write(self.run_q, False)

    def do_main_loop(self):
        '''
        Main loop. Starts listening to client server for various commands, starting and closing threads as necessary.
        '''
        # setup queues
        state_values = [self.strain.locked_read(), self.setpoint.locked_read(), self.cap.locked_read(), self.dl.locked_read(), self.l0_samp.locked_read(), self.voltage_1.locked_read(), self.voltage_2.locked_read(), self.output_1.locked_read(), self.output_2.locked_read(), self.p.locked_read(), self.i.locked_read(), self.d.locked_read(), self.min_voltage_1.locked_read(), self.min_voltage_2.locked_read(), self.max_voltage_1.locked_read(), self.max_voltage_2.locked_read(), self.slew_rate.locked_read(), self.ctrl_mode.locked_read(), self.ctrl_status.locked_read(), self.run.locked_read(), self.temperature.locked_read()]
        strain_q = Queue()
        setpoint_q = Queue()
        cap_q = Queue()
        dl_q = Queue()
        l0_samp_q = Queue()
        voltage_1_q = Queue()
        voltage_2_q = Queue()
        output_1_q = Queue()
        output_2_q = Queue()
        p_q = Queue()
        i_q = Queue()
        d_q = Queue()
        min_voltage_1_q = Queue()
        min_voltage_2_q = Queue()
        max_voltage_1_q = Queue()
        max_voltage_2_q = Queue()
        slew_rate_q = Queue()
        ctrl_mode_q = Queue()
        ctrl_status_q = Queue()
        run_q = Queue()
        temperature_q = Queue()
        queues = [strain_q, setpoint_q, cap_q, dl_q, l0_samp_q, voltage_1_q, voltage_2_q, output_1_q, output_2_q, p_q, i_q, d_q, min_voltage_1_q, min_voltage_2_q, max_voltage_1_q, max_voltage_2_q, slew_rate_q, ctrl_mode_q, ctrl_status_q, run_q, temperature_q]
        for ii, q in enumerate(queues):
            queue_write(q, state_values[ii])
        [self.strain_q, self.setpoint_q, self.cap_q, self.dl_q, self.l0_samp_q, self.voltage_1_q, self.voltage_2_q, self.output_1_q, self.output_2_q, self.p_q, self.i_q, self.d_q, self.min_voltage_1_q, self.min_voltage_2_q, self.max_voltage_1_q, self.max_voltage_2_q, self.slew_rate_q, self.ctrl_mode_q, self.ctrl_status_q, self.run_q, self.temperature_q] = queues

        self.initialize_instruments()

        # start monitoring lcr meter
        self.strain_monitor_loop = StoppableThread(target=self.start_strain_monitor)
        self.strain_monitor_loop.start()

        # create conntrol thread
        self.strain_control_loop = StoppableThread(target=self.start_strain_control, args=(self.ctrl_mode.locked_read(),))
        # why no self.strain_control_loop.start()? --Elizabeth

        # start comms
        self.comms_loop = StoppableThread(target=self.start_comms)
        self.comms_loop.start()

        if self.sim.locked_read()==False:
            # write log data to file every 1 second?
            self.filelog_loop = StoppableThread(target=self.filelog, args=(self.logging_interval.locked_read(),))
            self.filelog_loop.start()

        # infinite loop display
        display = StrainDisplay(queues)
        self.display_process = mp.Process(target=display.start_display)
        self.display_process.start()
        self.display_process.join()

        # join comm loop if it hasn't already been stopped. There is an issue here because unless the program is shut down by a comms event, the comms loop will be hung up listening...hmmm
        if self.comms_loop.is_alive():
            self.comms_loop.stop()
            self.comms_loop.join()

        # join file log loop if it hasn't already been stopped
        if self.sim.locked_read()==False:
            if self.filelog_loop.is_alive():
                self.filelog_loop.stop()
                self.filelog_loop.join()
        print('Strain server shutdown complete')

class StrainDisplay:

    def __init__(self, queues):

        # unpack queues
        [self.strain_q, self.setpoint_q, self.cap_q, self.dl_q, self.l0_samp_q, self.voltage_1_q, self.voltage_2_q, self.output_1_q, self.output_2_q, self.p_q, self.i_q, self.d_q, self.min_voltage_1_q, self.min_voltage_2_q, self.max_voltage_1_q, self.max_voltage_2_q, self.slew_rate_q, self.ctrl_mode_q, self.ctrl_status_q, self.run_q, self.temperature_q] = queues
        # setup dictionaries
        self.labels_dict = {"Sample Length (um)":self.l0_samp_q, "Setpoint":self.setpoint_q, "Strain":self.strain_q, "Capacitance (pF)":self.cap_q, "dL (um)":self.dl_q, "Voltage 1 (V)":self.voltage_1_q, "Voltage 2 (V)":self.voltage_2_q, "P":self.p_q, "I":self.i_q, "D":self.d_q, "Voltage 1 Min":self.min_voltage_1_q, "Voltage 1 Max":self.max_voltage_1_q, "Voltage 2 Min":self.min_voltage_2_q, "Voltage 2 Max":self.max_voltage_2_q, "Slew Rate":self.slew_rate_q, "Control Status":self.ctrl_status_q, "Control Mode":self.ctrl_mode_q, "Output 1":self.output_1_q, "Output_2":self.output_2_q, "Platform Temperature (K)":self.temperature_q}
        self.labels_val = []
        self.window=1000

    def start_display(self):
        '''
        initiate graphical display, based on pyQt and pyqtgraph.
        '''

        print('Starting GUI display')

        # setup qt window
        pg.setConfigOptions(antialias=True)
        bg_color = '#ffd788' #'#f6b8f9'
        self.app = QtWidgets.QApplication([])
        self.root = QtWidgets.QWidget()
        self.root.setWindowTitle('Strain Server')
        self.root.setStyleSheet(f'color:black; background-color:{bg_color}')

        # setup layouts
        spacing=20
        layout = QtWidgets.QHBoxLayout()
        layout_left = QtWidgets.QGridLayout()
        layout_right = QtWidgets.QVBoxLayout()
        layout_left.setSpacing(spacing)
        layout_right.setSpacing(spacing)
        layout.addLayout(layout_left)
        layout.addLayout(layout_right)

        # setup labels to queue dictionary and initialize
        for i, (name, q) in enumerate(self.labels_dict.items()):
            val = round(float(queue_read(q)),4)
            label_name = QtWidgets.QLabel(f"{name}:")
            label_val = QtWidgets.QLabel(f"{val}")
            self.labels_val.append(label_val)
            layout_left.addWidget(label_name, i, 0)
            layout_left.addWidget(label_val, i, 1)

        # setup plots
        plots = pg.GraphicsLayoutWidget(size=(1,1))
        plots.setBackground('w')
        layout_right.addWidget(plots)
        self.p11 = plots.addPlot()
        self.p12 = plots.addPlot()
        plots.nextRow()
        self.p21 = plots.addPlot()
        self.p22 = plots.addPlot()

        # setup axes
        for p in [self.p11,self.p12,self.p21,self.p22]:
            p.disableAutoRange()
            p.setLabel('bottom', 'time (s)')
        self.p11.setLabel('left', 'strain (a.u.)')
        self.p12.setLabel('left', r'dl (<font>&mu;m)')
        self.p21.setLabel('left', 'voltage 1 (V)')
        self.p22.setLabel('left', 'voltage 2 (V)')

        # define plot primitives
        self.time_vect = np.zeros(self.window)
        self.strain_vect = queue_read(self.strain_q)*np.ones(self.window)
        self.sp_vect = queue_read(self.setpoint_q)*np.ones(self.window)
        self.dl_vect = queue_read(self.dl_q)*np.ones(self.window)
        self.v1_vect = queue_read(self.voltage_1_q)*np.ones(self.window)
        self.v2_vect = queue_read(self.voltage_2_q)*np.ones(self.window)
        self.cap_vect = queue_read(self.cap_q)*np.ones(self.window)
        self.line11 = self.p11.plot(self.time_vect, self.strain_vect, pen=pg.mkPen('orange', width=4))
        self.line11_sp = self.p11.plot(self.time_vect, self.sp_vect, pen=pg.mkPen('black', width=4, style=QtCore.Qt.DashLine))
        self.line12 = self.p12.plot(self.time_vect, self.dl_vect, pen=pg.mkPen('blue', width=4))
        self.line21 = self.p21.plot(self.time_vect, self.v1_vect, pen=pg.mkPen('red', width=4))
        self.line22 = self.p22.plot(self.time_vect, self.v2_vect, pen=pg.mkPen('green', width=4))

        # start thread to update display
        self.t0 = time.time()
        self.j = 0
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update_display)
        print('Starting display update loop')
        timer.start(50)

        # run GUI
        self.root.setLayout(layout)
        self.root.show()
        self.app.exec()

        print('Shut down GUI display')

    def update_display(self):
        '''
        updates GUI plot
        '''
        values = np.zeros(len(self.labels_val))
        t_start = time.time()

        # update labels
        for i, (name, q) in enumerate(self.labels_dict.items()):
            val = queue_read(q)
            values[i] = val
            self.labels_val[i].setText(str(round(float(val),4)))

        # get new data - make more robust
        new_strain = values[2]
        new_dl = values[4]
        new_v1 = values[5]
        new_v2 = values[6]
        new_cap = values[3]
        new_sp = values[1]

        # update plot data
        self.time_vect[self.j] = t_start -self.t0
        self.strain_vect[self.j] = new_strain
        self.sp_vect[self.j] = new_sp
        self.dl_vect[self.j] = new_dl
        self.v1_vect[self.j] = new_v1
        self.v2_vect[self.j] = new_v2
        self.cap_vect[self.j] = new_cap
        indx = np.argsort(self.time_vect)
        self.line11.setData(self.time_vect[indx], self.strain_vect[indx])
        self.line11_sp.setData(self.time_vect[indx], self.sp_vect[indx])
        self.line12.setData(self.time_vect[indx], self.dl_vect[indx])
        self.line21.setData(self.time_vect[indx], self.v1_vect[indx])
        self.line22.setData(self.time_vect[indx], self.v2_vect[indx])

        # update axis limits
        #self.p11.autoRange()
        #self.p12.autoRange()
        #self.p21.autoRange()
        #self.p22.autoRange()
        t_lower, t_upper = self.find_axes_limits(np.min(self.time_vect), np.max(self.time_vect))
        s_lower, s_upper = self.find_axes_limits(min(np.min(self.strain_vect), np.min(self.sp_vect)), max(np.max(self.sp_vect), np.max(self.strain_vect)))
        dl_lower, dl_upper = self.find_axes_limits(np.min(self.dl_vect), np.max(self.dl_vect))
        v1_lower, v1_upper = self.find_axes_limits(np.min(self.v1_vect),np.max(self.v1_vect))
        v2_lower, v2_upper = self.find_axes_limits(np.min(self.v2_vect), np.max(self.v2_vect))
        for p in [self.p11, self.p12, self.p21, self.p22]:
            p.setXRange(t_lower, t_upper)
        self.p11.setYRange(s_lower, s_upper)
        self.p12.setYRange(dl_lower, dl_upper)
        self.p21.setYRange(v1_lower, v1_upper)
        self.p22.setYRange(v2_lower, v2_upper)

        self.j = (self.j + 1) % self.window

        if queue_read(self.run_q) == False:
            print('Shut down display update thread')
            self.app.quit()

    def find_axes_limits(self, lower, upper, fraction=0.1):
        '''
        helper function to obtain valid axes limits

        args:
            - lower:        lower limit
            - upper:        upper limit

        returns:
            - lower_valid:
            - upper_valid

        kwargs:
            - fraction:     fraction of lower/upper limit in data to offset min/max of plot. defaults to 0.1
        '''
        lower_valid = lower
        upper_valid = upper
        if np.isnan(lower):
            lower_valid = 0
        if np.isinf(lower):
            lower_valid = 0
        if np.isnan(upper):
            upper_valid = 0
        if np.isinf(upper):
            upper_valid = 0
        lower_valid = lower_valid - np.abs(lower_valid)*fraction
        upper_valid = upper_valid + np.abs(upper_valid)*fraction
        return lower_valid, upper_valid

if __name__=='__main__':
    '''
    Set SIM to True to simulate power supply and lcr meter for testing of code. Opens up communication channels and starts main server loop.
    '''

    if SIM==True:

        lcr = SimulatedLCR(0.808)
        ps = SimulatedPS(lcr)
        cryo = SimulatedMontana()

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))

            strainserver = StrainServer(lcr, ps, cryo, s, STARTING_SETPOINT, P, I, D, L0_SAMP, l0=L0, sim=SIM)

            strainserver.do_main_loop()

    else:

        # visa
        # rm = pyvisa.ResourceManager()
        # resources = rm.list_resources()
        # print(resources)

        with AgilentE4980(LCR_ADDRESS) as lcr:
            print(f'Connected to LCR meter at {LCR_ADDRESS}.')
            with razorbillRP100(PS_ADDRESS) as ps:
                print(f'Connected to power supply at {PS_ADDRESS}.')
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind((HOST, PORT))
                    print(f'Bound socket from host {HOST} to port {PORT}.')

                    cryo = cryocore.CryoCore(MONTANA_ADDRESS)

                    strainserver = StrainServer(lcr, ps, cryo, s, STARTING_SETPOINT, P, I, D, L0_SAMP, l0=L0, sim=SIM)

                    strainserver.do_main_loop()
