'''
An example notebook to demonstrate usage within a Jupyter Notebook.
'''

from strain_control.strain_client import StrainClient

sc = StrainClient()
#sc.transmit('0.075')
sc.start_strain_control()
sc.change_setpoint(-0.01)
sc.get_voltage(1)

import re
string = 'STR:-0.01'
re.match(r'STR:-?[0-9]+[\.]?[0-9]*', string)
re.search(r'-?[0-9]+[\.]?[0-9]*', string)
