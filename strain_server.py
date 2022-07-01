'''
Starts strain control server application, which coordiantes Keysight LCR meter and Razrbill RP100 power supply to control for strain on the Razorbill CSX130 strain cell.

The server operates over various threads: (1) a main GUI thread for displaying and interacting with instruments. (2) a control loop which include options for setting power supply output voltage to achieve a desired strain and PID control. (3) a monitoring loop for reading and logging instrument values. (4) a communications loop for recieving and responding to commands from the strain client via python's implementation of socket programming.

SOME IMPORTANT SAFETY NOTES:
- include proper voltage limits for the power supply (ideally as a function of temperature with some backups safety to ensure it defaults to lowest limits)
- wire both the power supply and the capacitor correctly by rereading appropriate sections in the manual

To Do:
- fix rough ramp -> PID transition
- come up with good way to closer server safely, setting voltages to 0 slowly, etc.
- how will we tell the server what the temperature is? I suppose it might be able to connect to the Lakeshore controller and get the tempreature that way?
- impelement safe socket communications?
- generalized control loop with various options
- expand gui to allow for control independent of server, show status, etc.
- PID loop tuning (preferably from the GUI)
- add documentation.
- reconcile simulated and physical voltage read calls.

'''

from threading_classes import LockedVar, StoppableThread
from simulation import SimulatedLCR, SimulatedPS
from pymeasure.instruments.agilent import AgilentE4980
from pymeasure.instruments.razorbill import razorbillRP100
import pyvisa
from simple_pid import PID
from threading import Thread, Lock, Event
import threading
import matplotlib.pyplot as plt
import numpy as np
import time
import sys
import socket
import re


##########################
### USER SETTINGS HERE ###
##########################
global SIM, STARTING_SETPOINT, P, I, D, L0, MAX_VOLTAGE, MIN_VOLTAGE, HOST, PORT, LCR_ADDRESS, PS_ADDRESS

SIM=True
STARTING_SETPOINT=0
P=1000
I=100
D=0.1
L0 = 68.68 # initial capacitor spacing

### LIMIT OUTPUT VOLTAGE HERE ###

MAX_VOLTAGE = 119 # V
MIN_VOLTAGE = -19 # V

### COMMUNICATION SETTINGS ###
LCR_ADDRESS = None
PS_ADDRESS = None
HOST = 'localhost'
PORT = 8888

###########################
###########################
###########################

class StrainServer:

    def __init__(self, lcr, ps, serversocket, setpoint, P, I, D, l0=68.68, sim=False):
        '''
        class constructor.

        args:
            - lcr                   pymeasure LCR handle or simulation object
            - ps:                   pymeasure power supply handle or simulation object
            - s:                    a bound socket for communicating with strain client.
            - setpoint(float):       initial setpoint for PID
            - P(float):              proportional PID parameter
            - I(float):              integral PID parameter
            - D(float):              derivative PID parameter

        returns: class instance object
        '''

        self.lcr = lcr
        self.ps = ps
        self.serversocket = s
        self.l0 = l0
        self.sim = sim
        strain, cap, imaginary_impedance, dl, _ = self.get_strain()
        self.strain = LockedVar(strain)
        self.setpoint = LockedVar(setpoint)
        self.cap = LockedVar(cap)
        self.imaginary_impedance = LockedVar(imaginary_impedance)
        self.dl = LockedVar(dl)
        v1, v2 = self.ps_read()
        self.voltage_1 = LockedVar(v1)
        self.voltage_2 = LockedVar(v2)
        self.pid = PID(P, I, D, setpoint=self.setpoint.locked_read())

    def initialize_instruments(self):
        '''
        Sets initial setting and paramters for both LCR meter and power supply.
        '''

        return 1

    def start_strain_control(self):
        '''
        High level handling of strain control. For now, sets new strain value by first slowly ramping voltage to approximate voltage and then maintaining strain with a restricted PID loop.
        '''

        current_setpoint = self.setpoint.locked_read() # hold initial setpoint
        pid_loop = StoppableThread(target=self.start_pid, args=(current_setpoint,), kwargs={'limit':False})
        print('setting rough voltage')
        #self.set_strain_rough(current_setpoint)
        print('rough voltage achieved, starting PID')
        pid_loop.start()

        current_thread = threading.current_thread()
        while current_thread.stopped()==False:
            new_setpoint = self.setpoint.locked_read()
            if new_setpoint != current_setpoint:
                current_setpoint = new_setpoint
                if pid_loop.is_alive():
                        pid_loop.stop()
                        pid_loop.join()
                        print('Stopped PID loop')
                pid_loop = StoppableThread(target=self.start_pid, args=(current_setpoint,), kwargs={'limit':False})
                #self.set_strain_rough(current_setpoint)
                pid_loop.start()
        pid_loop.stop()
        pid_loop.join()

    def start_strain_monitor(self):
        '''
        Continuously reads lcr meter and ps and updates all state variables to class instance variables. In addition, in the future this should handle logging of instrument data.
        '''

        current_thread = threading.current_thread()
        while current_thread.stopped() == False:
            strain, cap, imaginary_impedance, dl, _ = self.get_strain()
            v1, v2 = self.ps_read()
            self.strain.locked_update(strain)
            self.cap.locked_update(cap)
            self.imaginary_impedance.locked_update(imaginary_impedance)
            self.dl.locked_update(dl)
            self.voltage_1.locked_update(v1)
            self.voltage_2.locked_update(v2)

    def start_display(self):
        '''
        initiate plotting.
        '''

        # setup plots
        fig, [[ax11, ax12], [ax21, ax22]] = plt.subplots(2,2)
        fig.set_size_inches(10, 8)
        window = 1000
        nstep = 1
        ax11.set_ylabel('strain (a.u.)')
        ax11.set_xlabel('time (s)')
        ax12.set_ylabel('voltage 1 (V)')
        ax12.set_xlabel('time (s)')
        ax21.set_ylabel('voltage 2 (V)')
        ax21.set_xlabel('time (s)')
        ax22.set_ylabel('capacitance (pF)')
        ax22.set_xlabel('time (s)')

        time_vect = np.zeros(window) #np.linspace(0,1,window)
        strain_vect = np.zeros(window)
        v1_vect = np.zeros(window)
        v2_vect = np.zeros(window)
        cap_vect = np.zeros(window)
        line11, = ax11.plot(time_vect, strain_vect, 'o', ms=3, color='orange')
        line12, = ax12.plot(time_vect, v1_vect, 'o', ms=3, color='blue')
        line21, = ax21.plot(time_vect, v2_vect, 'o', ms=3, color='red')
        line22, = ax22.plot(time_vect, cap_vect, 'o', ms=3, color='green')
        plt.tight_layout()
        fig.canvas.draw()

        n = 0
        j = 0
        t0 = time.time()

        while True:
            n = n + 1
            i = n % nstep

            t = time.time() - t0
            new_strain = self.strain.locked_read()
            new_v1 = self.voltage_1.locked_read()
            #new_v2 = self.voltage_2.locked_read()
            new_v2 = self.setpoint.locked_read()
            new_cap = self.cap.locked_read()

            if i == 0:
                # update plot
                time_vect[j] = t
                strain_vect[j] = new_strain
                v1_vect[j] = new_v1
                v2_vect[j] = new_v2
                cap_vect[j] = new_cap
                j = (j + 1) % window

            line11.set_xdata(time_vect)
            line11.set_ydata(strain_vect)
            line12.set_xdata(time_vect)
            line12.set_ydata(v1_vect)
            line21.set_xdata(time_vect)
            line21.set_ydata(v2_vect)
            line22.set_xdata(time_vect)
            line22.set_ydata(cap_vect)
            ax11.set_xlim(np.min(time_vect), np.max(time_vect))
            ax12.set_xlim(np.min(time_vect), np.max(time_vect))
            ax21.set_xlim(np.min(time_vect), np.max(time_vect))
            ax22.set_xlim(np.min(time_vect), np.max(time_vect))
            ax11.set_ylim(np.min(strain_vect),np.max(strain_vect)*1.2)
            ax12.set_ylim(np.min(v1_vect),np.max(v1_vect)*1.2)
            ax21.set_ylim(np.min(v2_vect),np.max(v2_vect)*1.2)
            ax22.set_ylim(np.min(cap_vect),np.max(cap_vect)*1.2)
            plt.pause(0.05)
            fig.canvas.draw()
            fig.canvas.flush_events()

    def start_comms(self):
        '''
        start listening to serversocket and respond to connect requests with typical socket communications.
        '''

        current_thread = threading.current_thread()
        while current_thread.stopped()==False:

            self.serversocket.listen(1)
            print('Listening for strain client')
            conn, addr = self.serversocket.accept()

            with conn:
                print(f'Connected to strain client at address {addr}')
                while True: # run main loop
                    message = conn.recv(1024)
                    print('Receive from client initiated.')
                    if not message:
                        print('Message received and strain client terminated connection.')
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
                        print('Transmitting response to client')
                        conn.sendall(response.encode('utf8'))
                    except:
                        error_msg = 'Error: unable to transmit response to client.'
                        print(error_msg)
                        conn.sendall(error_msg.encode('utf8'))
                        break

    def set_strain_rough(self, setpoint):
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

        n=0
        while abs(strain_val) <= abs(setpoint_val):
            approx_voltage = start_voltage + n*voltage_increment
            self.ps_write(approx_voltage)

            loop_cond = True
            while loop_cond:
                v1, v2 = self.ps_read()
                if v1 > (approx_voltage - ps.tol) or v1 < (approx_voltage + ps.tol):
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

        print(f'Starting PID with setpoint {setpoint}')

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

    def ps_write(self, voltage):
        '''
        helper function to update power supply voltage within proper limits.

        args:
            - voltage(float):       voltage to set on power supply.

        returns: None

        '''

        # limit max/min voltage
        if voltage > MAX_VOLTAGE:
            voltage = MAX_VOLTAGE
        elif voltage < MIN_VOLTAGE:
            voltage = MIN_VOLTAGE

        # set voltages
        if self.sim==True:
            self.ps.set_voltage(voltage)
        else:
            self.ps.voltage_1 = voltage
            self.ps.voltage_2 = voltage

    def ps_read(self):
        '''
        returns voltage 1 and voltage 2 from power supply.

        args: None

        returns:
            - v1(float)
            - v2(float)s

        '''

        if self.sim==True:
            v1 = self.ps.voltage_1.locked_read()
            v2 = self.ps.voltage_2.locked_read()
        else:
            v1 = self.ps.voltage_1
            v2 = self.ps.voltage_2
        return v1, v2

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
        cap = impedance[0]
        imaginary_impedance = impedance[1]
        dl = self.capacitance_to_dl(cap)
        l = self.l0+dl
        strain = dl/self.l0
        return strain, cap, imaginary_impedance, dl, l

    def capacitance_to_dl(self, capacitance):
        '''
        helper function that returns change gap between sample plates from initial gap (dl = l - l0) given a capacitance reading based on the CS130 capacitor calibration.

        In the future will account for the tempreature offset. That is,  we must first do cap = cap - cap_offset and then can get length with the calibration curve.

        args:
            - capacitance(float):         capacitance in pF

        returns:
            - dl(float):                  l - l0, the change in gap between sample plates from initial value in um
        '''

        # capacitor specifications
        area = 5.95e6 # um^2
        l0 = 68.68 # um
        cap_initial_value = 0.8 # pF
        response = 12e-3 # pF/um
        eps0 = 8.854e-6 # pF/um - vacuum permitivity
        cap_offset = 0.04 # pf - eventually should obtain this from a temperature calibration of "0" strain.
        # temperature calibration curve: d = eps*A/(C - offset) - x0
        dl = eps0*area/(capacitance - cap_offset) - l0 # um
        return dl

    def strain_to_voltage(self, strain):
        '''
        Helper function that returns power supply voltage estimated for a desired strain.

        args:
            - strain(float):        desired strain

        returns:
            - voltage(float):       estimated required voltage to achieve strain
        '''

        l0 = 68.68
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

        if message=='SCTRL:':
            self.strain_control_loop.start()
            response = '1'
        elif message == 'ECTRL:':
            self.strain_control_loop.stop()
            response = '1'
        elif message == 'STR:?':
            response = str(self.strain.locked_read())
        elif re.match(r'STR:[0-9]+[\.]?[0-9]*', message):
            setpoint = float(re.search(r'[0-9]+[\.]?[0-9]*', message)[0])
            self.setpoint.locked_update(setpoint)
            response = '1'
        elif re.match(r'VOL[1-2]:\?', message):
            channel = int(re.search(r'[1-2]', message)[0])
            if channel==1:
                response = str(self.ps.voltage_1)
            elif channel==2:
                response = str(self.ps.voltage_2)
        elif re.match(r'VOL[1-2]:[0-9]+[\.]?[0-9]*', message):
            channel = int(re.search(r'[1-2]', message)[0])
            voltage = float(re.search(r'[0-9]+[\.]?[0-9]*', message)[1])
            self.ps_write(voltage) # change ps_write to specify channel as well.
            response = '1'
        elif re.match(r'VSLW:[0-9]+[\.]?[0-9]*', message):
            slew_rate = float(re.search(r'[0-9]+[\.]?[0-9]*', message)[0])
            self.ps.slew_rate = slew_rate
            response = '1'

        return response

    def do_test_loop(self):
        '''
        This test is meant to illustrate the usage of this package.
        '''
        self.initialize_instruments()

        self.setpoint.locked_update(0.02)

        # start monitoring lcr meter
        strain_read_loop = StoppableThread(target=self.start_strain_monitor)
        strain_read_loop.start()

        # start control loop
        strain_control_loop = StoppableThread(target=self.start_strain_control)
        #control_loop = StoppableThread(target=self.start_pid, args=(self.setpoint.locked_read(),))
        strain_control_loop.start()

        self.start_display()

        strain_control_loop.stop()
        strain_read_loop.stop()
        strain_control_loop.join()
        strain_read_loop.join()

    def do_main_loop(self):
        '''
        Main loop. Starts listening to client server for various commands, starting and closing threads as necessary.
        '''

        # this should not start any control explicitly, such that if anything fails and the server must be restarted, it starts off in a stable state.

        self.initialize_instruments()

        # start monitoring lcr meter
        self.strain_read_loop = StoppableThread(target=self.start_strain_monitor)
        self.strain_read_loop.start()

        # create conntrol thread
        self.strain_control_loop = StoppableThread(target=self.start_strain_control)
        #self.strain_control_loop.start()

        # start comms
        self.comms_loop = StoppableThread(target=self.start_comms)
        self.comms_loop.start()

        # infinite loop displaying strain
        self.start_display()

        # close everything
        if self.strain_read_loop.is_alive():
            self.strain_read_loop.stop()
        if self.strain_control_loop.is_alive():
            self.strain_read_loop.stop()
        if self.comms_loop.is_alive():
            self.comms_loop.stop()
        self.strain_read_loop.join()
        self.strain_control_loop.join()
        self.comms_loop.join()

if __name__=='__main__':
    '''
    Set SIM to True to simulate power supply and lcr meter for testing of code. Opens up communication channels and starts main server loop.
    '''

    if SIM==True:

        lcr = SimulatedLCR(0.808)
        ps = SimulatedPS(lcr)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))

            strainserver = StrainServer(lcr, ps, s, STARTING_SETPOINT, P, I, D, l0=L0, sim=SIM)

            # start test with command line argument '-test'
            args = sys.argv
            if len(args)>=2:
                if args[1]=='-test':
                    strainserver.do_test_loop()
            else:
                strainserver.do_main_loop()

    else:

        # visa
        rm = pyvisa.ResourceManager()
        resources = rm.list_resources()
        print(resources)

        with AgilentE4980(LCR_ADDRESS) as lcr:
            with razorbillRP100(PS_ADDRESS) as ps:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind((HOST, PORT))

                    strainserver = StrainServer(lcr, ps, s, STARTING_SETPOINT, P, I, D, l0=L0, sim=SIM)

                    # start test with command line argument '-test'
                    args = sys.argv
                    if args[1]=='-test':
                        strainserver.do_test_loop()
                    else:
                        strainserver.do_main_loop()
