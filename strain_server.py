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

#################################
### LIMIT OUTPUT VOLTAGE HERE ###
#################################
global MAX_VOLTAGE
global MIN_VOLTAGE
MAX_VOLTAGE = 119 # V
MIN_VOLTAGE = -19 # V

def initialize_instruments(lcr, ps):
    '''
    Sets initial setting and paramters for both LCR meter and power supply.

    args:
        - lcr                   pymeasure LCR handle
        - ps:                   pymeasure power supply handle

    returns: None
    '''

    return 1

def start_strain_control(lcr, ps, pid, setpoint, strain, l0=68.68, sim=False):
    '''
    Handles setting new strain value by first slowly ramping voltage to approximate voltage and then maintaining strain with a restricted PID loop.

    args:
        - lcr                   pymeasure LCR handle
        - ps:                   pymeasure power supply handle
        - setpoint:             LockedVar for holding the setpoint
        - strain:               LockedVar for holding the current strain

    returns: None
    '''

    current_setpoint = setpoint.locked_read() # hold initial setpoint
    pid_loop = StoppableThread(target=start_pid, args=(lcr, ps, pid, setpoint, strain), kwargs={'l0':l0, 'sim':sim, 'limit':True})
    print('setting rough voltage')
    set_strain_rough(lcr, ps, setpoint, strain, l0, sim)
    print('rough voltage achieve, starting PID')
    pid_loop.start()

    current_thread = threading.current_thread()
    while current_thread.stopped()==False:
        new_setpoint = setpoint.locked_read()
        if new_setpoint != current_setpoint:
            current_setpoint = new_setpoint
            if pid_loop.is_alive():
                    pid_loop.join()
            pid_loop = Thread(target=start_pid, args=(lcr, ps, pid, current_setpoint, strain), kwargs={'l0':l0, 'sim':sim, 'limit':True})
            set_strain_rough(lcr, ps, setpoint, strain, l0, sim)
            pid_loop.start()
    pid_loop.stop()

def start_strain_read(lcr, strain, l0=68.68):
    '''
    Continuously reads lcr meter and extracts strain. Intended to be run as a separate StoppableThread within the main loop.

    args:
        - lcr:
        - strain:   LockedVar for storing strain
        - l0:

    returns: None
    '''

    current_thread = threading.current_thread()
    while current_thread.stopped() == False:
        strain.locked_update(get_strain(lcr, l0)[0])

def set_strain_rough(lcr, ps, setpoint, strain, l0=68.68, sim=False):
    '''
    Ramp voltage on power supply to an approximately correct strain, returning once that strain has been achieved within tolerance. This can be proceeded by PID control.

    args:
        - lcr                       pymeasure LCR handle
        - ps:                       pymeasure power supply handle
        - setpoint(float):   a value of strain to aim for

    returns: None

    '''

    setpoint_val = setpoint.locked_read()
    strain_val = strain.locked_read()
    start_voltage = strain_to_voltage(setpoint_val)
    voltage_increment = 0.5
    strain_tol=0.005

    n=0
    while abs(strain_val) <= abs(setpoint_val):
        approx_voltage = start_voltage + n*voltage_increment
        ps_update(lcr, ps, approx_voltage, l0=l0, sim=sim)

        loop_cond = True
        while loop_cond:
            v1, v2 = ps_read(ps, sim)
            if v1 > (approx_voltage - ps.tol) or v1 < (approx_voltage + ps.tol):
                loop_cond = False
            strain_val = strain.locked_read()
            if abs(strain_val) >= abs(setpoint_val):
                loop_cond = False
        n=n+1

def start_pid(lcr, ps, pid, setpoint, strain, l0=68.68, sim=False, limit=False):
    '''
    Start PID loop to control strain.

    args:
        - lcr                   pymeasure LCR handle
        - ps:                   pymeasure power supply handle
        - pid:                  simple_pid PID class instance
        - setpoint:             LockedVar for holding the setpoint
        - strain:               LockedVar for holding the current strain

    returns: None

    kwargs:
        - limit(bool):
    '''
    if limit==True:
        v1, v2 = ps_read(ps, sim)
        v0 = v1

    current_thread = threading.current_thread()
    while current_thread.stopped()==False:
        # update setpoint
        pid.setpoint = setpoint.locked_read()
        # compute new output given current strain
        new_voltage = pid(strain.locked_read())
        #if limit==True:
        #     dv = new_voltage - v0
        #    if abs(dv) > 5:
        #        new_voltage = v0 + 5*(dv/abs(dv))
        # set the new output and get current value
        ps_update(lcr, ps, new_voltage, l0, sim)

def ps_update(lcr, ps, voltage, l0=68.68, sim=False):
    '''
    helper function to update power supply voltage within proper limits.

    args:
        - lcr                   pymeasure LCR handle
        - ps:                   pymeasure power supply handle
        - voltage(float):       voltage to set on power supply.

    returns: None

    kwargs:
        - l0:
        - sim:
    '''

    # limit max/min voltage
    if voltage > MAX_VOLTAGE:
        voltage = MAX_VOLTAGE
    elif voltage < MIN_VOLTAGE:
        voltage = MIN_VOLTAGE

    # set voltages
    if sim==True:
        ps.set_voltage(voltage)
    else:
        ps.voltage_1 = voltage
        ps.voltage_2 = voltage

def ps_read(ps, sim=False):
    '''
    returns voltage 1 and voltage 2

    args:
        - ps: power suppl
        - sim(bool):

    returns:
        - v1(float)
        - v2(float)s

    '''

    if sim==True:
        v1 = ps.voltage_1.locked_read()
        v2 = ps.voltage_2.locked_read()
    else:
        v1 = ps.voltage_1
        v2 = ps.voltage_2
    return v1, v2

def get_strain(lcr, l0=68.68):
    '''
    Querys LCR meter for current impedance measurement and uses calibration curve to return strain. Assumes that LCR measurement mode is set to one of the parallel modes, which is appropriate for measuring small capacitance (such as CPD)

    args:
        - lcr:                  pymeasure LCR handle
        - l0(float):            initial gap between sample plates in um

    returs:
        - strain(float):        calculated strain
        - l(float):             gap between sample plates in um
        - dl(float):            l - l0
    '''

    impedance = lcr.impedance # or read impedance as posted by another process
    cap = impedance[0]
    dl = capacitance_to_dl(cap)
    l = l0+dl
    strain = dl/l0
    return strain, cap, dl, l

def capacitance_to_dl(capacitance):
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

def strain_to_voltage(strain):
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

class LockedVar:
    '''
    Minimal class to implement a locking variable. Contains two private attributes, a value and a lock, and a few methods for safetly reading writing value via the lock.
    '''

    def __init__(self, val):
        self._value = val
        self._lock = Lock()

    def locked_read(self):
        with self._lock:
            return self._value

    def locked_update(self, val):
        with self._lock:
            self._value = val

class StoppableThread(Thread):
    '''
    Thread class with a stop() method. The thread itself has to check
    regularly for the stopped() condition.
    '''

    def __init__(self,  *args, **kwargs):
        super(StoppableThread, self).__init__(*args, **kwargs)
        self._stop_event = Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

class SimulatedLCR:
    '''
    simulation of LCR meter measuring CS130 capacitor for testing purposes.

    '''

    def __init__(self, val):
        self.impedance = [val, val] # complex impedance

class SimulatedPS:
    '''
    simulation of Razorbill power supply for testing purposes. Includes method v_to_imp(), which is meant to simulate the piezo response (and hence capacitor reading) expected for a given output voltage, such that when a new voltage is set the SimulatedLCR object responds accordingly.

    '''

    def __init__(self, lcr):
        self.voltage_1 = LockedVar(0)
        self.voltage_2 = LockedVar(0)
        self.slew_rate = LockedVar(10) # V/s
        self.tol = 0.01 # V
        self.lcr = lcr
        self.vmax = 120 # V
        self.vim = -20 # V
        self.voltage_setter = StoppableThread(target=self.set_new_voltage, args=(self.voltage_1.locked_read(),))

    def set_voltage(self, v):
        ''' use this for immediate update (no ramping)
        self.voltage_1 = v
        self.voltage_2 = v
        self.lcr.impedance = self.v_to_imp(v)
        '''
        if self.voltage_setter.is_alive():
            self.voltage_setter.stop()
        self.voltage_setter = StoppableThread(target=self.set_new_voltage, args=(v,))
        self.voltage_setter.start()

    def set_new_voltage(self, v): # incorporates ramp rate
        t0 = time.time()
        sign = 1
        v1 = self.voltage_1.locked_read()
        if v1 > v:
            sign = -1
        while v1 < (v-self.tol) or v1 > (v+self.tol):
            current_thread = threading.current_thread()
            if current_thread.stopped()==True:
                break
            t = time.time()
            dt = t-t0
            dv = sign*self.slew_rate.locked_read()*dt
            # only accept dv if it gets you closer to the setpoint
            if abs(v1 + dv) > abs(v):
                dv = v - v1
            v_new = v1 + dv
            v1 = v_new
            self.voltage_1.locked_update(v_new)
            self.voltage_2.locked_update(v_new)
            self.lcr.impedance = self.v_to_imp(v_new)
            t0 = t

    def v_to_imp(self, v):
        # 0.05 um/V (assuming linear relation between dl and v)
        dl = 0.05*v
        l0 = 68.68 # um
        area = 5.95e6 # um^2
        response = 12e-3 # pF/um - can be used for simpler approx
        eps0 = 8.854e-6 # pF/um - vacuum permitivity
        cap = eps0*area/(dl+l0) + 0.04
        return [cap, cap]

if __name__=='__main__':

    # start test with command line argument '-test'
    args = sys.argv
    if args[1]=='-test':
        '''
        This test is meant to illustrate the usage of this package. Set SIM to True to simulate power supply and lcr meter for testing of code.
        '''

        SIM=True

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
            initialize_instruments(lcr, ps)

        # variables to hold setpoint and strain value
        strain = LockedVar(0)
        setpoint = LockedVar(.075)

        strain_read_loop = StoppableThread(target=start_strain_read, args=(lcr, strain))
        strain_read_loop.start()

        # setup the PID loop
        pid = PID(1000, 100, .1, setpoint=setpoint.locked_read())
        # start PID control in a separate thread
        #pid_loop = StoppableThread(target=start_pid, args=(lcr, ps, pid, setpoint, strain), kwargs={'sim':True})
        control_loop = StoppableThread(target=start_strain_control, args=(lcr, ps, pid, setpoint, strain), kwargs={'sim':True})
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
            new_strain = strain.locked_read()
            new_v1 = ps.voltage_1.locked_read()
            new_v2 = ps.voltage_2.locked_read()
            new_cap = lcr.impedance[1]

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

    else:
        print('main loop')
