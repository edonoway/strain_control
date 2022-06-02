"""
Main file for controlling Razorbill CS130 strain cell.

Coordinates sensing capacitor from Keysight LCR meter and setting output voltage to piezzo stacks.
"""

from pymeasure.instruments.agilent import AgilentE4980
from pymeasure.instruments.razorbill import razorbillRP100
import pyvisa

# visa
rm = pyvisa.ResourceManager()
resources = rm.list_resources()
resources

inst1 = rm.open_resource(resources[1])

# setup connection with both instruments
lcr_address = ''
ps_address = ''
lcr = AgilentE4980(lcr_address)
ps = razorbillRP100(ps_address)
