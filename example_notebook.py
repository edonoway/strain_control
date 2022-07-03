'''
An example notebook to demonstrate usage within a Jupyter Notebook.
'''

from strain_control.strain_client import StrainClient

sc = StrainClient()
#sc.transmit('0.075')
sc.start_strain_control()
sc.change_setpoint(0.0)
sc.get_voltage(1)
sc.change_slew_rate(10)
sc.set_voltage(1,-10)

import re
string = 'SCTRL:1'
re.match(r'SCTRL:[1-3]', string)
re.search(r'[1-3]', string)
