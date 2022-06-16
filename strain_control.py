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


'''

from pymeasure.instruments.agilent import AgilentE4980
from pymeasure.instruments.razorbill import razorbillRP100
import pyvisa
from simple_pid import PID
from threading import Thread, Lock
import matplotlib.pyplot as plt
import numpy as np
import time

global MAX_VOLTAGE
global MIN_VOLTAGE
MAX_VOLTAGE = 119 # V
MIN_VOLTAGE = -19 # V

def start_strain_control(lcr, ps, pid, setpoint, strain):
    '''
    Handles setting new strain value by first slowly ramping voltage to approximate voltage and then maintaining strain with the PID loop. TBD if I will actually use this, but the idea is to reduce the possibility of sudden jumps in output voltage (perhaps the better way to do this is by tuning the PID).

    args:
        - lcr                   pymeasure LCR handle
        - ps:                   pymeasure power supply handle
        - setpoint:             LockedVar for holding the setpoint
        - strain:               LockedVar for holding the current strain

    returns: None
    '''

    current_setpoint = setpoint.locked_read() # hold initial setpoint
    pid_loop = Thread(target=start_pid, args=(lcr, ps, pid, setpoint, strain))
    set_strain_rough(lcr, ps, current_setpoint)
    pid_loop.start()

    while True:

        new_setpoint = setpoint.locked_read()
        if new_setpoint != current_setpoint:
            current_setpoint = new_setpoint
            if pid_loop.is_alive():
                    pid_loop.join()
            pid_loop = Thread(target=start_pid, args=(lcr, ps, pid, setpoint, strain))
            set_strain_rough(lcr, ps, current_setpoint)
            pid_loop.start()

def initialize_instruments(lcr, ps):
    '''
    Sets initial setting and paramters for both LCR meter and power supply

    args:
        - lcr: pymeasure instrument for LCR meter
        - ps:  pymeasure instrument for RP100 power supply
    '''

    return 1

def set_strain_rough(lcr, ps, strain_setpoint):
    '''
    Ramp voltage on power supply to an approximately correct voltage, returning once that voltage has been achieved or the strain setpoint has been exceeded. This should be proceeded by PID control.

    args:
        - lcr                       pymeasure LCR handle
        - ps:                       pymeasure power supply handle
        - strain_setpoint(float):   a value of strain to aim for

    returns: None

    '''

    approx_voltage = strain_to_voltage(strain)
    ps.voltage_1 = approx_voltage
    ps.voltage_2 = approx_voltage
    ps.set_voltage(approx_voltage)

    loop_cond = True
    while loop_cond:
        if ps.voltage_1 > (approx_voltage - ps.tol) or ps.voltage_1 <(approx_voltage + ps.tol):
            loop_cond = False
        if get_strain(lcr) > strain_setpoint:
            loop_cond = False

def start_pid(lcr, ps, pid, setpoint, strain, l0=68.68):
    '''
    Start PID loop to control strain.

    args:
        - lcr                   pymeasure LCR handle
        - ps:                   pymeasure power supply handle
        - pid:                  simple_pid PID class instance
        - setpoint:             LockedVar for holding the setpoint
        - strain:               LockedVar for holding the current strain

    returns: None
    '''

    while True:

        # update setpoint
        pid.setpoint = setpoint.locked_read()
        # compute new output given current strain
        new_voltage = pid(strain.locked_read())
        # set the new output and get current value
        control_update(lcr, ps, new_voltage, strain, l0=l0)

def control_update(lcr, ps, voltage, strain, l0=68.68):
    '''
    update PID output and get strain.

    args:
        - lcr                   pymeasure LCR handle
        - ps:                   pymeasure power supply handle
        - voltage(float):       voltage to set on power supply.
        - strain:               LockedVar holding strain.

    returns: None
    '''

    if voltage > MAX_VOLTAGE:
        voltage = MAX_VOLTAGE
        #print('Warning: maximum voltage exceeded, limiting output to '+str(voltage)+' V.')
    elif voltage < MIN_VOLTAGE:
        voltage = MIN_VOLTAGE
        #print('Warning: minimum voltage exceeded, limiting output to '+str(voltage)+' V.')
    # set voltages
    ps.voltage_1 = voltage
    ps.voltage_2 = voltage
    ps.set_voltage(voltage) # for testing purposes
    strain.locked_update(get_strain(lcr, l0=l0)[0])

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
    strain = dl/l # change to l0 then retune
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
        self.__value = val
        self.__lock = Lock()

    def locked_read(self):
        with self.__lock:
            return self.__value

    def locked_update(self, val):
        with self.__lock:
            self.__value = val

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
        self.voltage_1 = 0
        self.voltage_2 = 0
        self.slew_rate = 10 # V/s
        self.tol = 0.01 # V
        self.lcr = lcr
        self.voltage_setter = Thread(target=self.set_new_voltage, args=(self.voltage_1,))

    def set_voltage(self, v):
        self.voltage_1 = v
        self.voltage_2 = v
        self.lcr.impedance = self.v_to_imp(v)
        '''
        if self.voltage_setter.is_alive():
            #print('closing thread...')
            self.voltage_setter.join()
            #print('thread closed')
        #print('starting thread')
        self.voltage_setter = Thread(target=self.set_new_voltage, args=(v,))
        self.voltage_setter.start()
        #print('thread started')
        '''

    def set_new_voltage(self, v): # incorporates ramp rate
        t0 = time.time()
        sign = 1
        if self.voltage_1 > v:
            sign = -1
        while self.voltage_1 < (v-self.tol) or self.voltage_1 > (v+self.tol):
            t = time.time()
            dt = t-t0
            dv = sign*self.slew_rate*dt
            # only accept dv if it gets you closer to the setpoint
            if abs(self.voltage_1 + dv) > abs(v):
                dv = v - self.voltage_1
            v_new = self.voltage_1 + dv
            self.voltage_1 = v_new
            self.voltage_2 = v_new
            self.lcr.impedance = self.v_to_imp(v_new)
            t0 = t

    def stop_voltage(self):
        self.voltage_setter.join()

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
    '''
    This test program is meant to illustrate the usage of this package.
    '''
    '''
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
    '''

    lcr = SimulatedLCR(0.808)
    ps = SimulatedPS(lcr)

    if 1==0:

        ps.lcr = lcr
        ps.voltage_1 = 100
        ps.voltage_2 = 100
        ps.voltage_setter.start()
        ps.set_voltage(100)
        s = -1
        for i in range(100):
            #if i%20 == 0:
            #    print(i)
            #    ps.set_voltage(s*100, lcr)
            #    s = -s
            time.sleep(0.5)
            print(ps.voltage_1)

    if 1==1:
        # variables to hold setpoint and strain value
        strain0 = get_strain(lcr)[0]
        strain = LockedVar(strain0)
        setpoint = LockedVar(.075)

        print("initial strain: "+str(strain0))

        new_strain = strain.locked_read()
        print('\n')
        print('strain = '+str(new_strain))
        print('setpoint = '+str(setpoint.locked_read()))
        print('voltage out = '+str(ps.voltage_1))
        print('capacitance = '+str(lcr.impedance[1]))

        # setup the PID loop
        pid = PID(1500, 200, .1, setpoint=setpoint.locked_read())

        # start PID control in a separate thread
        pid_loop = Thread(target=start_pid, args=(lcr, ps, pid, setpoint, strain))
        pid_loop.start()


        # setup plots
        plt.ion()
        fig, ax = plt.subplots()
        window = 1000
        ax.set_ylabel('strain')
        ax.set_xlabel('time')

        time_vect = [i for i in range(window)]
        strain_vect = np.zeros(window)
        line, = ax.plot(time_vect, strain_vect)

        n = 0
        while True:
            n = n + 1
            i = n % window

            # ask user to change setpoint
            time.sleep(0.5)

            new_strain = strain.locked_read()
            print('\n')
            print('strain = '+str(new_strain))
            print('setpoint = '+str(setpoint.locked_read()))
            print('voltage out = '+str(ps.voltage_1))
            print('capacitance = '+str(lcr.impedance[1]))

            # update plot
            strain_vect[i] = new_strain
            line.set_ydata(strain_vect)
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.show()

        pid_loop.join()
