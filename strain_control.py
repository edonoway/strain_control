"""
Main file for controlling Razorbill CS130 strain cell.

Coordinates sensing capacitor from Keysight LCR meter and setting output voltage to piezzo stacks into a PID loop. For convenience, may want to set up as a class.
"""

from pymeasure.instruments.agilent import AgilentE4980
from pymeasure.instruments.razorbill import razorbillRP100
import pyvisa
from simple_pid import PID
from threading import Thread, Lock
import matplotlib.pyplot as plt
import numpy as np
import time


def start_strain_pid(lcr, ps, pid, setpoint, strain):
    '''
    Start PID loop to control strain.

    NOTE: The proper design for this PID loop is still not clear. The goal is to start a process that will robustly maintain a given strain, and yet where strain can easily be changed within a parent process.

    I think actually the best way to design this is as is already implemented for other intruments, such as the temperature controller. There, the PID loop is run as a process within the Lakeshore controller - ie, in a process external to the control process. There is some variable that gets set by the control process to change setpoint, which is then read by the PID process. I wonder if I can do something similar, where this PID process looks somewhere for a setpoint variable and then tries to maintain that.

    The other requirement is to be able to easily record the meaured strain.

    I also wonder if the right way to go is first slowly ramp up to approximate correct voltage as given by dl = 0.05(um/V)*V to get the desired strain, and then once voltage is achieved, start PID control about that voltage? 

    SOME IMPORTANT SAFETY NOTES:
    - include proper voltage limits for the power supply (ideally as a function of temperature with some backups safety to ensure it defaults to lowest limits)
    - wire both the power supply and the capacitor correctly by rereading appropriate sections in the manual

    Args:
        - lcr:  pymeasure object for LCR meter
        - ps:   pymeasure object for power supply
        - pid:  simple_pid PID class instance to manage PID set values
        - setpoint: LockedVar for holding the setpoint
        - strain:   LockedVar for holding the current strain

    Rerturns:
        - None
    '''

    while True:

        # update setpoint
        pid.setpoint = setpoint.locked_read()

        # compute new output given current strain
        new_voltage = pid(strain.locked_read())
        # set the new output and get current value
        strain_update(lcr, ps, new_voltage, strain)

def strain_update(lcr, ps, voltage, strain):
    '''
    update PID output and get strain.

    Args:
        - lcr
        - ps
        - voltage(float):
        - strain:     LockedVar
    '''

    # set voltages
    ps.voltage_1 = voltage
    ps.voltage_2 = voltage
    ps.set_voltage(voltage, lcr) # for testing purposes

    strain.locked_update(get_strain(lcr))

def initialize_instruments(lcr, ps):
    '''
    Sets initial setting and paramters for both LCR meter and power supply

    args:
        - lcr: pymeasure instrument for LCR meter
        - ps:  pymeasure instrument for RP100 power supply
    '''

    return 1

def get_strain(lcr, l0=68.68):
    '''
    Querys LCR meter for current impedance measurement and uses calibration curve to return strain. Assumes that LCR measurement mode is set to one of the parallel modes, which is appropriate for measuring small capacitance (such as CPD)

    args:
        - lcr: pymeasure instrument for LCR meter
        - l0:  initial gap in um

    returs:
        - strain(float): calculated strain
    '''

    impedance = lcr.impedance # or read impedance as posted by another process
    cap = impedance[0]
    dl = capacitance_to_dl(cap)
    l = l0+dl
    strain = dl/l
    return strain

def capacitance_to_dl(capacitance):
    '''
    helper function that returns change in length given a capacitance reading based on the CS130 capacitor calibration, which in the future will account for the tempreature offset. That is,  we must first do cap = cap - cap_offset and then can get length with the calibration curve.
    '''

    # capacitor specifications
    area = 5.95e6 # um^2
    initial_gap = 68.68 # um
    initial_value = 0.8 # pF
    response = 12e-3 # pF/um
    eps0 = 8.854e-6 # pF/um - vacuum permitivity
    offset = 0.04 # pf - eventually should obtain this from a temperature calibration of "0" strain.

    # temperature calibration curve: d = eps*A/(C - offset) - x0
    gap = eps0*area/(capacitance - offset) - initial_gap # um
    return gap

class LockedVar:
    '''
    minimal class to implement a locking variable.
    '''

    def __init__(self, val):
        self.value = val
        self._lock = Lock()

    def locked_read(self):
        with self._lock:
            return self.value

    def locked_update(self, val):
        with self._lock:
            self.value = val

# a few classes to fake the instruments
class FakeLCR:

    def __init__(self, val):
        self.impedance = [val, val] # complex impedance

class FakePS:

    def __init__(self):
        self.voltage_1 = 0
        self.voltage_2 = 0

    def set_voltage(self, v, lcr):
        self.voltage_1 = v
        self.voltage_2 = v
        lcr.impedance = self.v_to_imp(v)

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

    lcr = FakeLCR(0.808)
    ps = FakePS()

    # variables to hold setpoint and strain value
    strain0 = get_strain(lcr)
    strain = LockedVar(strain0)
    setpoint = LockedVar(.1)

    print("initial strain: "+str(get_strain(lcr)))

    new_strain = strain.locked_read()
    print('\n')
    print('strain = '+str(new_strain))
    print('setpoint = '+str(setpoint.locked_read()))
    print('voltage out = '+str(ps.voltage_1))
    print('capacitance = '+str(lcr.impedance[1]))

    # setup the PID loop
    pid = PID(1500, 200, .1, setpoint=setpoint.locked_read())

    # start PID control in a separate thread
    pid_loop = Thread(target=start_strain_pid, args=(lcr, ps, pid, setpoint, strain))
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
