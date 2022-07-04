'''
An example notebook to demonstrate usage within a Jupyter Notebook.
'''

from strain_control.strain_client import StrainClient

sc = StrainClient()
#sc.transmit('0.075')
sc.start_strain_control('PID')
sc.change_setpoint(0.075)
sc.get_voltage(1)
sc.get_strain()
sc.change_slew_rate(5)
sc.set_voltage(1,100)
sc.stop_strain_control()
sc.shutdown_server()
