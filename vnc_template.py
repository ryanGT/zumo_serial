import zumo_serial
import serial_utils
from numpy import *
import numpy, time

class myclass_p(zumo_serial.zumo_serial_connection_pid_control):
    def __init__(self, *args, **kwargs):
        zumo_serial.zumo_serial_connection_pid_control.__init__(self, \
                                                        *args, **kwargs)
        self.new_parameter = 7.0#<-- just an example parameter
        

    def calc_v(self, q):
        v_diff = self.kp*self.error[q]


if __name__ == '__main__':
    my_zumo = myclass_p(kp=0.25, kd=2.0)
    # use these commands to run the test:
    # 1. my_zumo.open_serial()
    # 2. my_zumo.calibrate()
    # 3. my_zumo.run_test()
    # 4. my_zumo.report_numpy()
    # 5. my_zumo.plot()
    
    
