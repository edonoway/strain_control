'''
Main file for controlling Razorbill CS130 strain cell.

Coordinates sensing capacitor from Keysight LCR meter and setting output voltage to piezzo stacks into a PID loop. For convenience, may want to set up as a class.

NOTE: The proper design for this PID loop is still not clear. The goal is to start a process that will robustly maintain a given strain, and yet where strain can easily be changed within a parent process.

I think actually the best way to design this is as is already implemented for other intruments, such as the temperature controller. There, the PID loop is run as a process within the Lakeshore controller - ie, in a process external to the control process. There is some variable that gets set by the control process to change setpoint, which is then read by the PID process. I wonder if I can do something similar, where this PID process looks somewhere for a setpoint variable and then tries to maintain that.

The other requirement is to be able to easily record the meaured strain.

I also wonder if the right way to go is first slowly ramp up to approximate correct voltage as given by dl = 0.05(um/V)*V to get the desired strain, and then once voltage is achieved, start PID control about that voltage?

SOME IMPORTANT SAFETY NOTES:
- include proper voltage limits for the power supply (ideally as a function of temperature with some backups safety to ensure it defaults to lowest limits)
- wire both the power supply and the capacitor correctly by rereading appropriate sections in the manual


Two issues: (1) getting PID to play with rough ramp, (2) robustly updating strain object rather than coupling it with writing.


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



##########################
### USER SETTINGS HERE ###
##########################
global SIM, STARTING_SETPOINT, P, I, D, L0, MAX_VOLTAGE, MIN_VOLTAGE

SIM=True
STARTING_SETPOINT=0.075
P=1000
I=100
D=0.1
L0 = 68.68 # initial capacitor spacing

### LIMIT OUTPUT VOLTAGE HERE ###

MAX_VOLTAGE = 119 # V
MIN_VOLTAGE = -19 # V

###########################
###########################
###########################

class StrainServer:

    def __init__(self, lcr, ps, setpoint, P, I, D, l0=68.68, sim=False):
        '''
        class constructor.

        args:
            - lcr                   pymeasure LCR handle or simulation object
            - ps:                   pymeasure power supply handle or simulation object
            - setpoint(float):       initial setpoint for PID
            - P(float):              proportional PID parameter
            - I(float):              integral PID parameter
            - D(float):              derivative PID parameter

        returns: class instance object
        '''

        self.lcr = lcr
        self.ps = ps
        self.l0 = l0
        self.sim = sim
        self.strain = LockedVar(self.get_strain()[0])
        self.setpoint = LockedVar(setpoint)
        self.pid = PID(P, I, D, setpoint=self.setpoint.locked_read())

    def initialize_instruments(self):
        '''
        Sets initial setting and paramters for both LCR meter and power supply.

        args: None

        returns: None
        '''

        return 1

    def start_strain_control(self):
        '''
        Handles setting new strain value by first slowly ramping voltage to approximate voltage and then maintaining strain with a restricted PID loop.

        args: None

        returns: None
        '''

        current_setpoint = self.setpoint.locked_read() # hold initial setpoint
        pid_loop = StoppableThread(target=self.start_pid, args=(current_setpoint,))
        print('setting rough voltage')
        self.set_strain_rough()
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
                pid_loop = Thread(target=self.start_pid, args=(current_setpoint,))
                self.set_strain_rough()
                pid_loop.start()
        pid_loop.stop()
        pid_loop.join()

    def start_strain_read(self):
        '''
        Continuously reads lcr meter and extracts strain. Intended to be run as a separate StoppableThread within the main loop.

        args: None

        returns: None
        '''

        current_thread = threading.current_thread()
        while current_thread.stopped() == False:
            self.strain.locked_update(self.get_strain()[0])

    def set_strain_rough(self):
        '''
        Ramp voltage on power supply to an approximately correct strain, returning once that strain has been achieved within tolerance. This can be proceeded by PID control.

        args: None

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
            self.ps_update(approx_voltage)

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

        self.pid.setpoint = setpoint

        current_thread = threading.current_thread()
        while current_thread.stopped()==False:

            # compute new output given current strain
            new_voltage = self. pid(self.strain.locked_read())
            #if limit==True:
            #     dv = new_voltage - v0
            #    if abs(dv) > 5:
            #        new_voltage = v0 + 5*(dv/abs(dv))
            # set the new output and get current value
            self.ps_update(new_voltage)

    def ps_update(self, voltage):
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
        dl = self.capacitance_to_dl(cap)
        l = self.l0+dl
        strain = dl/self.l0
        return strain, cap, dl, l

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

    def do_test_loop(self):
        '''
        This test is meant to illustrate the usage of this package.
        '''
        self.initialize_instruments()

        # start monitoring lcr meter
        strain_read_loop = StoppableThread(target=self.start_strain_read)
        strain_read_loop.start()

        # start control loop
        control_loop = StoppableThread(target=self.start_strain_control)
        control_loop.start()

        # setup plots
        fig, [[ax11, ax12], [ax21, ax22]] = plt.subplots(2,2)
        window = 10000
        nstep = 2000
        ax11.set_ylabel('strain (a.u.)')
        ax11.set_xlabel('time (s)')
        ax12.set_ylabel('voltage 1 (V)')
        ax12.set_xlabel('time (s)')
        ax21.set_ylabel('voltage 2 (V)')
        ax21.set_xlabel('time (s)')
        ax22.set_ylabel('capacitance (pF)')
        ax22.set_xlabel('time (s)')

        time_vect = np.zeros(window)
        strain_vect = np.zeros(window)
        v1_vect = np.zeros(window)
        v2_vect = np.zeros(window)
        cap_vect = np.zeros(window)
        line11, = ax11.plot(time_vect, strain_vect)
        line12, = ax12.plot(time_vect, v1_vect)
        line21, = ax21.plot(time_vect, v2_vect)
        line22, = ax22.plot(time_vect, cap_vect)

        n = 0
        j = 0
        t0 = time.time()
        while j < window:
            n = n + 1
            i = n % nstep

            t = time.time() - t0
            new_strain = self.strain.locked_read()
            new_v1 = self.ps.voltage_1.locked_read()
            new_v2 = self.ps.voltage_2.locked_read()
            new_cap = self.lcr.impedance[1]

            if i == 0:
                # update plot
                time_vect[j] = t
                strain_vect[j] = new_strain
                v1_vect[j] = new_v1
                v2_vect[j] = new_v2
                cap_vect[j] = new_cap
                j = j + 1

        control_loop.stop()
        strain_read_loop.stop()
        control_loop.join()
        strain_read_loop.join()

        ax11.plot(time_vect, strain_vect, 'o', ms=0.5, color='orange')
        ax12.plot(time_vect, v1_vect, 'o', ms=0.5, color='blue')
        ax21.plot(time_vect, v2_vect, 'o', ms=0.5, color='red')
        ax22.plot(time_vect, cap_vect, 'o', ms=0.5, color='green')
        plt.tight_layout()
        plt.show()

    def do_main_loop(self):

        return 1

if __name__=='__main__':
    '''
    Set SIM to True to simulate power supply and lcr meter for testing of code.
    '''

    # initiate conections to lcr and ps
    if SIM==True:
        lcr = SimulatedLCR(0.808)
        ps = SimulatedPS(lcr)

    else:
        # visa
        rm = pyvisa.ResourceManager()
        resources = rm.list_resources()
        resources
        inst1 = rm.open_resource(resources[1])
        # setup connection with both instruments and initialize
        lcr_address = ''
        ps_address = ''
        lcr = AgilentE4980(lcr_address)
        ps = razorbillRP100(ps_address)

    # generate server object
    strainserver = StrainServer(lcr, ps, STARTING_SETPOINT, P, I, D, l0=L0, sim=SIM)

    # start test with command line argument '-test'
    args = sys.argv
    if args[1]=='-test':
        strainserver.do_test_loop()
    else:
        strainserver.do_main_loop()
