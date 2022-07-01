'''
Strain client indended to be run from jupyter notebook.
'''
import socket

##########################
### USER SETTINGS HERE ###
##########################
global MAX_VOLTAGE, MIN_VOLTAGE, HOST, PORT

### LIMIT OUTPUT VOLTAGE HERE ###

MAX_VOLTAGE = 119 # V
MIN_VOLTAGE = -19 # V

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

    def query(self, message):
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

        print(f'Received: {response}')
        return response.decode('utf8')

    def start_strain_control(self):

        return 1

    def change_setpoint(self, new_setpoint):

        return 1

    def set_voltage(self, axis, voltage):

        return 1

    def read_strain(self):

        return 1

    def maintain_voltage(self):

        return 1

    def change_ramp_rate(self, new_rr):

        return 1
