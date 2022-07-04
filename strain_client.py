'''
Strain client indended to be run from jupyter notebook.
'''
import socket

##########################
### USER SETTINGS HERE ###
##########################
global HOST, PORT

### COMMUNICATION SETTINGS ###
HOST = 'localhost'
PORT = 8888

###########################
###########################
###########################


class StrainClient:

    def __init__(self):
         self.host = HOST
         self.port = PORT

    def transmit(self, message):
        '''
        Implement basic query of strain server.

        args:
            - message(string):      message to send to strain server

        returns:
            - response:             response from server.

        '''

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.connect((self.host, self.port))
            except:
                raise RuntimeError('unable to connect to strain server.')
            try:
                message = str(message).encode('utf-8')
                s.sendall(message)
            except:
                raise RuntimeError('unable to send query to strain server.')
            try:
                response = s.recv(1024)
            except:
                raise RuntimeError('unable to recieve response from strain server.')

        #print(f'Received: {response}')
        return response.decode('utf8')

    def start_strain_control(self, mode='PID'):
        '''
        initiate control loop on strain server.

        returns:
            - response:

        kwargs:
            - mode(string):     'PID', 'Set Voltage', or 'Combined'
        '''
        if mode not in ['PID', 'Set Voltage', 'Combined']:
            raise ValueError('invalid control mode, please input PID, Set Voltage, or Combine.')
        if mode=='PID':
            code=1
        elif mode=='Set Voltage':
            code=2
        elif mode=='Combined':
            code=3
        message = 'SCTRL:'+str(code)
        response = self.transmit(message)
        return response

    def stop_strain_control(self):
        '''
        stop control loop on strain server.

        returns:
            - response:
        '''

        message = 'ECTRL:'
        response = self.transmit(message)
        return response

    def get_strain(self):
        '''
        Get current strain from cell.

        args: None

        returns:
            - strain(float):        strain
        '''

        message = 'STR:?'
        strain = float(self.transmit(message))
        return strain

    def get_dl(self):
        '''
        Get current gap from cell.

        args: None

        returns:
            - dl(float):        strain
        '''

        message = 'DL:?'
        dl = float(self.transmit(message))
        return dl

    def get_cap(self):
        '''
        Get current capacitance from cell.

        args: None

        returns:
            - cap(float):        strain
        '''

        message = 'CAP:?'
        cap = float(self.transmit(message))
        return cap

    def get_voltage(self, channel):
        '''
        read voltage on given channel.

        args:
            - channel(int):     channel 1 or 2 on power supply

        returns:
            - voltage(float):
        '''

        if not(channel==1 or channel==2):
            raise ValueError('Invalid power supply voltage channel, please choose either 1 or 2.')
        message = 'VOL'+str(channel)+':?'
        voltage = float(self.transmit(message))
        return voltage

    def set_setpoint(self, new_setpoint):
        '''
        change target strain setpoint of control loop.

        args:
            - new_setpoint:     setpoint to set

        returns:
            - response:
        '''

        message = 'STR:'+str(new_setpoint)
        response = self.transmit(message)
        return response

    def set_voltage(self, channel, voltage):
        '''
        sets voltage explicitly on channel 1 or 2

        args:
            - channel(int):     channel 1 or 2 on power supply
            - voltage(float):   voltage to set

        returns:
            - response:
        '''

        if not(channel==1 or channel==2):
            raise ValueError('Invalid power supply voltage channel, please choose either 1 or 2.')
        message = 'VOL'+str(channel)+':'+str(voltage)
        response = self.transmit(message)
        return response

    def set_pid(self, p, i, d):
        '''
        Set PID parameters.

        args:
            - p(float):
            - i(float):
            - d(float):

        returns:
            - response(str):
        '''
        message = f'PID:{p},{i},{d}'
        response = self.transmit(message)
        return response

    def set_slew_rate(self, slew_rate):
        '''
        change voltage ramp rate on power supply:

        args:
            - slew_rate(float):    slew rate in V/s

        returns:
            - response:
        '''

        message = 'VSLW:'+str(slew_rate)
        response = self.transmit(message)
        return response

    def shutdown_server(self):
        '''
        Terminates strain server, correctly shutting down the system and leaving it in a stable, safe state.
        '''

        message = 'SHTDWN'
        response = self.transmit(message)
        return response
