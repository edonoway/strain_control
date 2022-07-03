'''
classes and methods for simulating the strain control system, mainly for testing code.
'''

from threading_classes import LockedVar, StoppableThread
import threading
import time

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
        self.voltage_1 = LockedVar(0) # tension stack
        self.voltage_2 = LockedVar(0) # compression stack
        self.slew_rate = LockedVar(10) # V/s
        self.tol = 0.01 # V
        self.lcr = lcr
        self.vmax = 120 # V
        self.vim = -20 # V
        self.voltage_setter1 = StoppableThread(target=self.set_new_voltage, args=(1, self.voltage_1.locked_read(),))
        self.voltage_setter2 = StoppableThread(target=self.set_new_voltage, args=(2, self.voltage_2.locked_read(),))

    def set_voltage(self, channel, v):
        ''' use this for immediate update (no ramping)
        self.voltage_1 = v
        self.voltage_2 = v
        self.lcr.impedance = self.v_to_imp(v)
        '''
        if channel==1:
            if self.voltage_setter1.is_alive():
                self.voltage_setter1.stop()
            self.voltage_setter1 = StoppableThread(target=self.set_new_voltage, args=(channel, v))
            self.voltage_setter1.start()
        elif channel==2:
            if self.voltage_setter2.is_alive():
                self.voltage_setter2.stop()
            self.voltage_setter2 = StoppableThread(target=self.set_new_voltage, args=(channel, v))
            self.voltage_setter2.start()

    def set_new_voltage(self, channel, v): # incorporates ramp rate
        if channel==1:
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
                # only accept dv if it doesn't overshoot
                if sign==1:
                    if v1 + dv > v:
                        dv = v - v1
                elif sign==-1:
                    if v1 + dv < v:
                        dv = v - v1
                v_new = v1 + dv
                v1 = v_new
                self.voltage_1.locked_update(v_new)
                self.lcr.impedance = self.v_to_imp(v_new)
                t0 = t
            if channel==2:
                t0 = time.time()
                sign = 1
                v2 = self.voltage_2.locked_read()
                if v2 > v:
                    sign = -1
                while v2 < (v-self.tol) or v2 > (v+self.tol):
                    current_thread = threading.current_thread()
                    if current_thread.stopped()==True:
                        break
                    t = time.time()
                    dt = t-t0
                    dv = sign*self.slew_rate.locked_read()*dt
                    # only accept dv if it gets you closer to the setpoint
                    if sign==1:
                        if v2 + dv > v:
                            dv = v - v2
                    elif sign==-1:
                        if v2 + dv < v:
                            dv = v - v2
                    v_new = v1 + dv
                    v2 = v_new
                    self.voltage_2.locked_update(v_new)
                    #self.lcr.impedance = self.v_to_imp(v_new)
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
