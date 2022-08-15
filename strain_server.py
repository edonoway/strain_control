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

from threading_classes import LockedVar, StoppableThread, StoppableProcess, queue_write
from simulation import SimulatedLCR, SimulatedPS
from pymeasure.instruments.agilent import AgilentE4980
from pymeasure.instruments.razorbill import razorbillRP100
import pyvisa
from simple_pid import PID
from threading import Thread, Lock, Event
import threading
import multiprocessing as mp
from multiprocessing import Queue
import matplotlib.pyplot as plt
import numpy as np
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

### LIMIT OUTPUT VOLTAGE HERE ###

MAX_VOLTAGE = 119#119 # V
MIN_VOLTAGE = -5#-19 # V

### COMMUNICATION SETTINGS ###
LCR_ADDRESS = 'USB0::0x2A8D::0x2F01::MY54412905::0::INSTR'
PS_ADDRESS = 'ASRL4::INSTR'
HOST = 'localhost'
PORT = 15200

###########################
###########################
###########################

def start_display(queues):
    '''
    initiate plotting.

    args:
      -queue(mp.Queue):       a queue pip for receiving updated data
    '''

    print('Starting GUI display')

    # setup qt window
    pg.setConfigOptions(antialias=True)
    bg_color = '#ffd788' #'#f6b8f9'
    app = QtWidgets.QApplication([])
    root = QtWidgets.QWidget()
    root.setWindowTitle('Strain Server')
    root.setStyleSheet(f'color:black; background-color:{bg_color}')

    # setup layouts
    layout = QtWidgets.QHBoxLayout()
    layout_left = QtWidgets.QGridLayout()
    layout_right = QtWidgets.QVBoxLayout()
    layout.addLayout(layout_left)
    layout.addLayout(layout_right)

    # setup labels
    [strain_q, setpoint_q, cap_q, dl_q, l0_samp_q, voltage_1_q, voltage_2_q, p_q, d_q, min_voltage_1_q, min_voltage_2_q, max_voltage_1_q, max_voltage_2_q, slew_rate_q, ctrl_mode_q] = queues
    labels_dict = {"Sample Length (um)":l0_samp_q, "Setpoint":setpoint_q, "Strain":strain_q, "Capacitance (pF)":cap_q, "dL (um)":dl_q, "Voltage 1 (V)":voltage_1_q, "Voltage 2 (V)":voltage_2_q, "P":p_q, "I":i, "D":d_q, "Voltage 1 Min":min_voltage_1_q, "Voltage 1 Max":max_voltage_1_q, "Voltage 2 Min":min_voltage_2_q, "Voltage 2 Max":max_voltage_2_q, "Slew Rate":slew_rate_q, "Control Status":ctrl_mode_q}
    labels_val = []
    for i, (name, q) in enumerate(labels_dict.items()):
        val = round(queue_read(q),4)
        label_name = QtWidgets.QLabel(f"{name}:")
        label_val = QtWidgets.QLabel(f"{val}")
        labels_val.append(label_val)
        layout_left.addWidget(label_name, i, 0)
        layout_left.addWidget(label_val, i, 1)

    # setup subplots
    plots = pg.GraphicsLayoutWidget()
    plots.setBackground('w')
    layout_right.addWidget(plots)
    p11 = plots.addPlot()
    p12 = plots.addPlot()
    plots.nextRow()
    p21 = plots.addPlot()
    p22 = plots.addPlot()

    # setup axes
    for p in [p11,p12,p21,p22]:
        p.disableAutoRange()
        p.setLabel('bottom', 'time', units='s')
    p11.setLabel('left', 'strain (a.u.)')
    p12.setLabel('left', r'dl ($\mu$m)')
    p21.setLabel('left', 'voltage 1 (V)')
    p22.setLabel('left', 'voltage 2 (V)')

    # define plot primitives
    window = 10000
    time_vect = np.zeros(window)
    strain_vect = np.zeros(window)
    sp_vect = np.zeros(window)
    dl_vect = np.zeros(window)
    v1_vect = np.zeros(window)
    v2_vect = np.zeros(window)
    cap_vect = np.zeros(window)
    """
    line11 = p11.plot(time_vect, strain_vect, pen=None, symbolBrush='orange', symbol='o',symbolSize=5)
    line11_sp = p11.plot(time_vect, sp_vect, pen=pg.mkPen('black', width=3, style=QtCore.Qt.DashLine))
    line12 = p12.plot(time_vect, dl_vect, pen=None, symbolBrush='blue', symbol='o', symbolSize=5)
    line21 = p21.plot(time_vect, v1_vect, pen=None, symbolBrush='red', symbol='o', symbolSize=5)
    line22 = p22.plot(time_vect, v2_vect, pen=None, symbolBrush='green', symbol='o', symbolSize=5)
    """
    line11 = p11.plot(time_vect, strain_vect, pen=pg.mkPen('orange', width=4))
    line11_sp = p11.plot(time_vect, sp_vect, pen=pg.mkPen('black', width=4, style=QtCore.Qt.DashLine))
    line12 = p12.plot(time_vect, dl_vect, pen=pg.mkPen('blue', width=4))
    line21 = p21.plot(time_vect, v1_vect, pen=pg.mkPen('red', width=4))
    line22 = p22.plot(time_vect, v2_vect, pen=pg.mkPen('green', width=4))

    # start thread to update display
    update_thread = StoppableThread(target=update_display, args=(p11, p12, p21, p22, time_vect, strain_vect, sp_vect, dl_vect, v1_vect, v2_vect, cap_vect, line11, line11_sp, line12, line21, line22, window, labels_dict, labels_val))
    update_thread.start()

    # run GUI
    root.setLayout(layout)
    root.show()
    app.exec()

    print('Shut down GUI display')

    # for safety, check if run condition still true and shutdown if true
    #if self.run.get()==True:
    #    self.shutdown(1)

def update_display(p11, p12, p21, p22, time_vect, strain_vect, sp_vect, dl_vect, v1_vect, v2_vect, cap_vect, line11, line11_sp, line12, line21, line22, window, labels_dict, labels_val):
    '''
    updates GUI plot
    '''

    print('Starting display update loop')

    time_vect_local, strain_vect_local, sp_vect_local, dl_vect_local, v1_vect_local, v2_vect_local, cap_vect_local = time_vect, strain_vect, sp_vect, dl_vect, v1_vect, v2_vect, cap_vect

    values = np.zeros(len(labels_val))

    t0 = time.time()
    j = 0
    current_thread = threading.current_thread()
    while current_thread.stopped() == False:

        t_start = time.time()

        # update labels
        for i, (name, q) in enumerate(labels_dict.items()):
            val = queue_read(q)
            values[i] = val
            labels_val[i].setText(round(str(val),4))

        # get new data
        # strain_q, setpoint_q, cap_q, dl_q, l0_samp_q, voltage_1_q, voltage_2_q, p_q, d_q, min_voltage_1_q, min_voltage_2_q, max_voltage_1_q, max_voltage_2_q, slew_rate_q, ctrl_mode_q
        new_strain = values[0]
        new_dl = values[3]
        new_v1 = values[5]
        new_v2 = values[6]
        new_cap = values[2]
        new_sp = values[1]
        new_p = values[7]
        new_i = values[8]
        new_d = values[9]

        # update plot data
        time_vect_local[j] = t_start - t0
        strain_vect_local[j] = new_strain
        sp_vect_local[j] = new_sp
        dl_vect_local[j] = new_dl
        v1_vect_local[j] = new_v1
        v2_vect_local[j] = new_v2
        cap_vect_local[j] = new_cap
        indx = np.argsort(time_vect_local)
        line11.setData(time_vect_local[indx], strain_vect_local[indx])
        line11_sp.setData(time_vect_local[indx], sp_vect_local[indx])
        line12.setData(time_vect_local[indx], dl_vect_local[indx])
        line21.setData(time_vect_local[indx], v1_vect_local[indx])
        line22.setData(time_vect_local[indx], v2_vect_local[indx])

        # update axis limits
        t_lower, t_upper = self.find_axes_limits(np.min(time_vect_local), np.max(time_vect_local))
        s_lower, s_upper = self.find_axes_limits(min(np.min(strain_vect_local)*0.8, np.min(sp_vect_local)*0.8), max(np.max(sp_vect_local)*1.2, np.max(strain_vect_local)*1.2))
        dl_lower, dl_upper = self.find_axes_limits(np.min(dl_vect_local)*0.8, np.max(dl_vect_local)*1.2)
        v1_lower, v1_upper = self.find_axes_limits(np.min(v1_vect_local)*0.8,np.max(v1_vect_local)*1.2)
        v2_lower, v2_upper = self.find_axes_limits(np.min(v2_vect_local)*0.8, np.max(v2_vect_local)*1.2)
        for p in [p11, p12, p21, p22]:
            p.setXRange(t_lower, t_upper)
        p11.setYRange(s_lower, s_upper)
        p12.setYRange(dl_lower, dl_upper)
        p21.setYRange(v1_lower, v1_upper)
        p22.setYRange(v2_lower, v2_upper)

        j = (j + 1) % window

        #t_end = time.time()
        #print(t_end-t_start)
        #time.sleep(0.1)

    print('Shut down display update thread')

class StrainServer:

    def __init__(self, lcr, ps, serversocket, setpoint, p, i, d, l0_samp, l0=68.68, sim=False):
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
            - l0_samp(float):        initial length of sample
            - l0(float):             initial capacitor gap

        returns: class instance object
        '''

        self.lcr = lcr
        self.ps = ps
        self.serversocket = s
        self.l0 = l0
        self.l0_samp = LockedVar(l0_samp)
        self.sim = sim
        strain, cap, imaginary_impedance, dl = self.get_strain()
        self.strain = LockedVar(strain)
        self.setpoint = LockedVar(setpoint)
        self.cap = LockedVar(cap)
        self.imaginary_impedance = LockedVar(imaginary_impedance)
        self.dl = LockedVar(dl)
        v1, v2 = self.get_voltage(1), self.get_voltage(2)
        self.voltage_1 = LockedVar(v1)
        self.voltage_2 = LockedVar(v2)
        self.max_voltage_1 = LockedVar(MAX_VOLTAGE)
        self.min_voltage_1 = LockedVar(MIN_VOLTAGE)
        self.max_voltage_2 = LockedVar(MAX_VOLTAGE)
        self.min_voltage_2 = LockedVar(MIN_VOLTAGE)
        self.slew_rate = LockedVar(SLEW_RATE)
        self.pid = PID(p, i, d, setpoint=self.setpoint.locked_read())
        #self.pid.sample_time = 0.01
        self.p, self.i, self.d = [LockedVar(j) for j in self.pid.tunings]
        self.ctrl_mode = LockedVar(1)
        self.run = LockedVar(True)
        self.host = HOST
        self.port = PORT

        # setup queues
        values = [strain, setpoint, cap, dl, l0_samp, v1, v2, p, i, d, MIN_VOLTAGE, MIN_VOLTAGE, MAX_VOLTAGE, MAX_VOLTAGE, SLEW_RATE, self.ctrl_mode.locked_read()]
        self.strain_q = Queue()
        self.setpoint_q = Queue()
        self.cap_q = Queue()
        self.dl_q = Queue()
        self.l0_samp_q = Queue()
        self.voltage_1_q = Queue()
        self.voltage_2_q = Queue()
        self.p_q = Queue()
        self.i_q = Queue()
        self.d_q = Queue()
        self.min_voltage_1_q = Queue()
        self.min_voltage_2_q = Queue()
        self.max_voltage_1_q = Queue()
        self.max_voltage_2_q = Queue()
        self.slew_rate_q = Queue()
        self.ctrl_mode_q = Queue()
        self.queues = [self.strain_q, self.setpoint_q, self.cap_q, self.dl_q, self.l0_samp_q, self.voltage_1_q, self.voltage_2_q, self.p_q, self.i_q, self.d_q, self.min_voltage_1_q, self.min_voltage_2_q, self.max_voltage_1_q, self.max_voltage_2_q, self.slew_rate_q, self.ctrl_mode_q]
        for ii, q in enumerate(self.queues):
            q.put(values[ii])

    def initialize_instruments(self):
        '''
        Sets initial setting and paramters for both LCR meter and power supply.
        '''

        self.set_slew_rate(SLEW_RATE)

        return 1

    def start_strain_control(self, mode):
        '''
        High level handling of strain control. For now, sets new strain value by first slowly ramping voltage to approximate voltage and then maintaining strain with a restricted PID loop.

        args:
            - mode(int):     1:'PID, 2:'Set Voltage', or 3:'Combined'

        returns: None
        '''

        current_setpoint = self.setpoint.locked_read()

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

        print('Shut down control thread')

    def start_strain_monitor(self):
        '''
        Continuously reads lcr meter and ps and updates all state variables to class instance variables. In addition, in the future this should handle logging of instrument data.
        '''

        print('Starting strain monitor')

        current_thread = threading.current_thread()
        while current_thread.stopped() == False:
            strain, cap, imaginary_impedance, dl = self.get_strain()
            v1 = self.get_voltage(1)
            v2 = self.get_voltage(2)
            self.strain.locked_update(strain)
            self.cap.locked_update(cap)
            self.imaginary_impedance.locked_update(imaginary_impedance)
            self.dl.locked_update(dl)
            self.voltage_1.locked_update(v1)
            self.voltage_2.locked_update(v2)

            # update queues
            queues = [self.strain_q, self.cap_q, self.dl_q, self.voltage_1_q, self.voltage_2_q]
            vars = [strain, cap, dl, v1, v2]
            for ii, q in enumerate(queues):
                queue_write(q, vars[ii])

        print('Shut down monitor thread')

    def start_display(self):
        '''
        initiate plotting.

        args:
          -queue(mp.Queue):       a queue pip for receiving updated data
        '''

        print('Starting GUI display')
        queues = self.queues

        # setup qt window
        pg.setConfigOptions(antialias=True)
        bg_color = '#ffd788' #'#f6b8f9'
        app = QtWidgets.QApplication([])
        root = QtWidgets.QWidget()
        root.setWindowTitle('Strain Server')
        root.setStyleSheet(f'color:black; background-color:{bg_color}')

        # setup layouts
        layout = QtWidgets.QHBoxLayout()
        layout_left = QtWidgets.QGridLayout()
        layout_right = QtWidgets.QVBoxLayout()
        layout.addLayout(layout_left)
        layout.addLayout(layout_right)

        # setup labels
        [strain_q, setpoint_q, cap_q, dl_q, l0_samp_q, voltage_1_q, voltage_2_q, p_q, d_q, min_voltage_1_q, min_voltage_2_q, max_voltage_1_q, max_voltage_2_q, slew_rate_q, ctrl_mode_q] = queues
        labels_dict = {"Sample Length (um)":l0_samp_q, "Setpoint":setpoint_q, "Strain":strain_q, "Capacitance (pF)":cap_q, "dL (um)":dl_q, "Voltage 1 (V)":voltage_1_q, "Voltage 2 (V)":voltage_2_q, "P":p_q, "I":i, "D":d_q, "Voltage 1 Min":min_voltage_1_q, "Voltage 1 Max":max_voltage_1_q, "Voltage 2 Min":min_voltage_2_q, "Voltage 2 Max":max_voltage_2_q, "Slew Rate":slew_rate_q, "Control Status":ctrl_mode_q}
        labels_val = []
        for i, (name, q) in enumerate(labels_dict.items()):
            val = round(queue_read(q),4)
            label_name = QtWidgets.QLabel(f"{name}:")
            label_val = QtWidgets.QLabel(f"{val}")
            labels_val.append(label_val)
            layout_left.addWidget(label_name, i, 0)
            layout_left.addWidget(label_val, i, 1)

        # setup subplots
        plots = pg.GraphicsLayoutWidget()
        plots.setBackground('w')
        layout_right.addWidget(plots)
        p11 = plots.addPlot()
        p12 = plots.addPlot()
        plots.nextRow()
        p21 = plots.addPlot()
        p22 = plots.addPlot()

        # setup axes
        for p in [p11,p12,p21,p22]:
            p.disableAutoRange()
            p.setLabel('bottom', 'time', units='s')
        p11.setLabel('left', 'strain (a.u.)')
        p12.setLabel('left', r'dl ($\mu$m)')
        p21.setLabel('left', 'voltage 1 (V)')
        p22.setLabel('left', 'voltage 2 (V)')

        # define plot primitives
        window = 10000
        time_vect = np.zeros(window)
        strain_vect = np.zeros(window)
        sp_vect = np.zeros(window)
        dl_vect = np.zeros(window)
        v1_vect = np.zeros(window)
        v2_vect = np.zeros(window)
        cap_vect = np.zeros(window)
        """
        line11 = p11.plot(time_vect, strain_vect, pen=None, symbolBrush='orange', symbol='o',symbolSize=5)
        line11_sp = p11.plot(time_vect, sp_vect, pen=pg.mkPen('black', width=3, style=QtCore.Qt.DashLine))
        line12 = p12.plot(time_vect, dl_vect, pen=None, symbolBrush='blue', symbol='o', symbolSize=5)
        line21 = p21.plot(time_vect, v1_vect, pen=None, symbolBrush='red', symbol='o', symbolSize=5)
        line22 = p22.plot(time_vect, v2_vect, pen=None, symbolBrush='green', symbol='o', symbolSize=5)
        """
        line11 = p11.plot(time_vect, strain_vect, pen=pg.mkPen('orange', width=4))
        line11_sp = p11.plot(time_vect, sp_vect, pen=pg.mkPen('black', width=4, style=QtCore.Qt.DashLine))
        line12 = p12.plot(time_vect, dl_vect, pen=pg.mkPen('blue', width=4))
        line21 = p21.plot(time_vect, v1_vect, pen=pg.mkPen('red', width=4))
        line22 = p22.plot(time_vect, v2_vect, pen=pg.mkPen('green', width=4))

        # start thread to update display
        self.update_thread = StoppableThread(target=self.update_display, args=(p11, p12, p21, p22, time_vect, strain_vect, sp_vect, dl_vect, v1_vect, v2_vect, cap_vect, line11, line11_sp, line12, line21, line22, window, labels_dict, labels_val))
        self.update_thread.start()

        # run GUI
        root.setLayout(layout)
        root.show()
        app.exec()

        print('Shut down GUI display')

        # for safety, check if run condition still true and shutdown if true
        #if self.run.get()==True:
        #    self.shutdown(1)

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

    def update_display(self, p11, p12, p21, p22, time_vect, strain_vect, sp_vect, dl_vect, v1_vect, v2_vect, cap_vect, line11, line11_sp, line12, line21, line22, window, labels_dict, labels_val):
        '''
        updates GUI plot
        '''

        print('Starting display update loop')

        time_vect_local, strain_vect_local, sp_vect_local, dl_vect_local, v1_vect_local, v2_vect_local, cap_vect_local = time_vect, strain_vect, sp_vect, dl_vect, v1_vect, v2_vect, cap_vect

        values = np.zeros(len(labels_val))

        t0 = time.time()
        j = 0
        current_thread = threading.current_thread()
        while current_thread.stopped() == False:

            t_start = time.time()

            # update labels
            for i, (name, q) in enumerate(labels_dict.items()):
                val = queue_read(q)
                values[i] = val
                labels_val[i].setText(round(str(val),4))

            # get new data
            # strain_q, setpoint_q, cap_q, dl_q, l0_samp_q, voltage_1_q, voltage_2_q, p_q, d_q, min_voltage_1_q, min_voltage_2_q, max_voltage_1_q, max_voltage_2_q, slew_rate_q, ctrl_mode_q
            new_strain = values[0]
            new_dl = values[3]
            new_v1 = values[5]
            new_v2 = values[6]
            new_cap = values[2]
            new_sp = values[1]
            new_p = values[7]
            new_i = values[8]
            new_d = values[9]

            # update plot data
            time_vect_local[j] = t_start - t0
            strain_vect_local[j] = new_strain
            sp_vect_local[j] = new_sp
            dl_vect_local[j] = new_dl
            v1_vect_local[j] = new_v1
            v2_vect_local[j] = new_v2
            cap_vect_local[j] = new_cap
            indx = np.argsort(time_vect_local)
            line11.setData(time_vect_local[indx], strain_vect_local[indx])
            line11_sp.setData(time_vect_local[indx], sp_vect_local[indx])
            line12.setData(time_vect_local[indx], dl_vect_local[indx])
            line21.setData(time_vect_local[indx], v1_vect_local[indx])
            line22.setData(time_vect_local[indx], v2_vect_local[indx])

            # update axis limits
            t_lower, t_upper = self.find_axes_limits(np.min(time_vect_local), np.max(time_vect_local))
            s_lower, s_upper = self.find_axes_limits(min(np.min(strain_vect_local)*0.8, np.min(sp_vect_local)*0.8), max(np.max(sp_vect_local)*1.2, np.max(strain_vect_local)*1.2))
            dl_lower, dl_upper = self.find_axes_limits(np.min(dl_vect_local)*0.8, np.max(dl_vect_local)*1.2)
            v1_lower, v1_upper = self.find_axes_limits(np.min(v1_vect_local)*0.8,np.max(v1_vect_local)*1.2)
            v2_lower, v2_upper = self.find_axes_limits(np.min(v2_vect_local)*0.8, np.max(v2_vect_local)*1.2)
            for p in [p11, p12, p21, p22]:
                p.setXRange(t_lower, t_upper)
            p11.setYRange(s_lower, s_upper)
            p12.setYRange(dl_lower, dl_upper)
            p21.setYRange(v1_lower, v1_upper)
            p22.setYRange(v2_lower, v2_upper)

            j = (j + 1) % window

            #t_end = time.time()
            #print(t_end-t_start)
            #time.sleep(0.1)

        print('Shut down display update thread')

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

            # update setpoint
            #self.pid.setpoint = self.setpoint.locked_read()

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
            if self.sim==True:
                self.ps.set_voltage(channel, voltage)
            else:
                if channel==1:
                    self.ps.voltage_1 = voltage
                elif channel==2:
                    self.ps.voltage_2 = voltage
            #print(f'Ramping voltage on channel {channel} to {voltage} V')
        except:
            print('Error: invaid voltage channel, please choose 1 or 2.')

    def get_voltage(self, channel):
        '''
        returns voltage 1 or voltage 2 from power supply.

        args: None

        returns:
            - v1(float)
            - v2(float)s

        '''
        try:
            if not (channel==1 or channel==2):
                raise ValueError('channel must be int 1 or 2.')

            if self.sim==True:
                if channel==1:
                    v = self.ps.voltage_1.locked_read()
                elif channel==2:
                    v = self.ps.voltage_2.locked_read()
            else:
                if channel==1:
                    v = self.ps.voltage_1
                elif channel==2:
                    v = self.ps.voltage_2
            return v
        except:
            print('Error: invaid voltage channel, please choose 1 or 2.')

    def set_slew_rate(self, slew_rate):
        '''
        utility to set slew rate on power supply on both channels
        '''

        print(f'Setting slew rate on power supply to {slew_rate} V/s')
        if SIM==True:
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
        cap = impedance[0]
        imaginary_impedance = impedance[1]
        dl = self.capacitance_to_dl(cap)
        strain = dl/self.l0_samp.locked_read()
        return strain, cap, imaginary_impedance, dl

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
        l0 = self.l0 # um
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
                    queue_write(self.ctrl_mode, mode)
                    self.strain_control_loop.stop()
                    self.strain_control_loop.join()
                    self.strain_control_loop = StoppableThread(target=self.start_strain_control, args=(mode,))
                    self.strain_control_loop.start()
                else:
                    print(f'Control thread in mode {mode} already in progress, no action taken')
            else:
                print(f'Starting control thread in mode {mode}')
                self.ctrl_mode.locked_update(mode)
                queue_write(self.ctrl_mode, mode)
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
            queue_write(self.l0_samp, samp_l0)
            response='1'
        elif re.match(r'PID:[0-9]+[\.]?[0-9]*,[0-9]+[\.]?[0-9]*,[0-9]+[\.]?[0-9]*', message):
            p, i, d = [float(j) for j in re.findall(r'[0-9]+[\.]?[0-9]*', message)]
            self.pid.tunings = (p,i,d)
            self.p.locked_update(p)
            self.i.locked_update(i)
            self.d.locked_update(d)
            queu_write(self.p, p)
            queu_write(self.i, i)
            queu_write(self.d, d)
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

    def find_axes_limits(self, lower, upper):
        """
        helper function to obtain valid axes limits

        args:
            - lower:        lower limit
            - upper:        upper limit

        returns:
            - lower_valid:
            - upper_valid
        """

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

        return lower_valid, upper_valid

    def close_display(self):
        '''
        closes the display safely.
        '''

        # stop display update loop
        if self.update_thread.is_alive():
            self.update_thread.stop()
            self.update_thread.join()

        self.app.quit()

    def shutdown(self, mode):
        '''
        Initiates shutdown of server.

        args:
            - mode(int):        0 to leave state of system as is, or 1 to ramp voltages down to 0.

        returns: None
        '''

        print('Shutting down strain server:')
        self.run.locked_update(False)
        if self.comms_loop.is_alive():
            self.comms_loop.stop()
            # can't join because we might be in it!
        if mode==1:
            print('Ramping voltage on all channels to 0')
            self.set_voltage(1, 0)
            self.set_voltage(2, 0)
            eps = 0.1
            while (np.abs(self.get_voltage(1)) >= eps) or (np.abs(self.get_voltage(1)) >= eps):
                continue
        if self.strain_control_loop.is_alive():
            self.strain_control_loop.stop()
            self.strain_control_loop.join()
        if self.strain_monitor_loop.is_alive():
            self.strain_monitor_loop.stop()
            self.strain_monitor_loop.join()
        self.close_display()

    def do_main_loop(self):
        '''
        Main loop. Starts listening to client server for various commands, starting and closing threads as necessary.
        '''

        # this should not start any control explicitly, such that if anything fails and the server must be restarted, it starts off in a stable state.

        self.initialize_instruments()

        # start monitoring lcr meter
        self.strain_monitor_loop = StoppableThread(target=self.start_strain_monitor)
        self.strain_monitor_loop.start()

        # create conntrol thread
        self.strain_control_loop = StoppableThread(target=self.start_strain_control, args=(self.ctrl_mode.locked_read(),))

        # start comms
        self.comms_loop = StoppableThread(target=self.start_comms)
        self.comms_loop.start()

        # infinite loop display
        self.display_process = mp.Process(target=self.start_display)
        self.display_process.start()
        self.display_process.join()
        #while self.run.locked_read()==True:
        #    continue

        # join comm loop if it hasn't already been stopped. There is an issue here because unless the program is shut down by a comms event, the comms loop will be hung up listening...hmmm
        if self.comms_loop.is_alive():
            self.comms_loop.stop()
            self.comms_loop.join()
        print('Strain server shutdown complete')

if __name__=='__main__':
    '''
    Set SIM to True to simulate power supply and lcr meter for testing of code. Opens up communication channels and starts main server loop.
    '''

    if SIM==True:

        lcr = SimulatedLCR(0.808)
        ps = SimulatedPS(lcr)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))

            strainserver = StrainServer(lcr, ps, s, STARTING_SETPOINT, P, I, D, L0_SAMP, l0=L0, sim=SIM)

            strainserver.do_main_loop()

    else:

        # visa
        # rm = pyvisa.ResourceManager()
        # resources = rm.list_resources()
        # print(resources)

        with AgilentE4980(LCR_ADDRESS) as lcr:
            print('Connected to LCR meter.')
            with razorbillRP100(PS_ADDRESS) as ps:
                print('Connected to power supply.')
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind((HOST, PORT))
                    print(f'Bound socket from host {HOST} to port {PORT}.')

                    strainserver = StrainServer(lcr, ps, s, STARTING_SETPOINT, P, I, D, L0_SAMP, l0=L0, sim=SIM)

                    strainserver.do_main_loop()
