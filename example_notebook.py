'''
An example notebook to demonstrate usage within a Jupyter Notebook.
'''

from strain_control.strain_client import StrainClient

sc = StrainClient()
#sc.transmit('0.075')
sc.start_strain_control('PID')
sc.set_setpoint(0.075)
sc.get_voltage(1)
sc.get_strain()
sc.set_slew_rate(5)
sc.set_voltage(1,100)
sc.set_pid(1000,100,0.1)
sc.stop_strain_control()
sc.shutdown_server()

import re
message = 'PID:1000,1.0,100'
re.findall(r'[0-9]+[\.]?[0-9]*', message)
