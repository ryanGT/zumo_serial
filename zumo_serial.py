from matplotlib.pyplot import *
#from scipy import *
from numpy import *
import numpy, time

#import control

import time, copy, os

import serial_utils

#righthand side for me
#portname = '/dev/tty.usbmodem1411'
#lefthand side for me

# do I want to reconnect the serial each time like this?

#time.sleep(0.1)

dt = 1.0/100#<---------- is this true for your choice of OCR1A?

class zumo_serial_connection_ol(object):
    def __init__(self, ser=None, mymin=0, mymax=400, \
                 numsensors = 6):
        self.ser = ser
        self.min = mymin
        self.max = mymax
        self.numsensors = numsensors
        

    def open_serial(self):
        # check if it is open first
        from myserial import ser

        time.sleep(3.0)# <--- will this work under windows?
                       # does this work better with a Z32U4
                       # if it isn't rebooting?
        serial_utils.WriteByte(ser, 0)
        debug_line = serial_utils.Read_Line(ser)
        line_str = ''.join(debug_line)
        self.ser = ser
        return line_str


    def flush(self):
        self.ser.flushInput()
        self.ser.flushOutput()


    def close(self):
        serial_utils.Close_Serial(self.ser)


    def calibrate(self):
        serial_utils.WriteByte(self.ser, 4)#start new test
        check_4 = serial_utils.Read_Byte(self.ser)
        return check_4

    
    def get_error(self):
        serial_utils.WriteByte(self.ser,5)
        e_out = serial_utils.Read_Two_Bytes_Twos_Comp(self.ser)
        return e_out


    def mysat(self, m_in):
        if m_in < self.min:
            return self.min
        elif m_in > self.max:
            return self.max
        else:
            return int(m_in)
    

    def run_test(self, uL, uR):
        serial_utils.WriteByte(self.ser, 2)#start new test
        check_2 = serial_utils.Read_Byte(self.ser)

        N = len(uL)

        nvect = zeros(N,dtype=int)
        numsensors = 6
        sensor_mat = zeros((N,numsensors))
        error = zeros_like(nvect)

        self.nvect = nvect
        self.uL = uL
        self.uR = uR
        self.sensor_mat = sensor_mat
        self.error = error


        for i in range(N):
            serial_utils.WriteByte(self.ser, 1)#new n and voltage are coming
            serial_utils.WriteInt(self.ser, i)
            serial_utils.WriteInt(self.ser, uL[i])
            serial_utils.WriteInt(self.ser, uR[i])

            nvect[i] = serial_utils.Read_Two_Bytes(self.ser)
            for j in range(numsensors):
                sensor_mat[i,j] = serial_utils.Read_Two_Bytes_Twos_Comp(self.ser)
            error[i] = serial_utils.Read_Two_Bytes_Twos_Comp(self.ser)
            nl_check = serial_utils.Read_Byte(self.ser)
            assert nl_check == 10, "newline problem"


        serial_utils.WriteByte(self.ser, 3)#stop test
        check_3 = serial_utils.Read_Byte(self.ser)
        print('check_3 = ' + str(check_3))

        return nvect, sensor_mat, error


    def plot(self, fignum=1):
        figure(fignum)
        clf()
        plot(self.nvect, self.error)

        figure(fignum+1)
        clf()
        plot(self.nvect, self.uL, self.nvect, self.uR)

        figure(fignum+2)
        clf()
        for i in range(self.numsensors):
            plot(self.nvect, self.sensor_mat[:,i])


        show()


    def append_plot(self, fignum):
        figure(fignum)
        plot(self.nvect, self.error)



class zumo_serial_connection_p_control(zumo_serial_connection_ol):
    def __init__(self, ser=None, kp=0.1, nominal_speed=400, \
                 **kwargs):
        zumo_serial_connection_ol.__init__(self, ser=ser, **kwargs)
        self.kp = kp
        self.nominal_speed = nominal_speed
        

    def calc_v(self, q, error):
        v = error[q]*self.kp
        return v


    def run_test(self, N=200):
        serial_utils.WriteByte(self.ser, 2)#start new test
        check_2 = serial_utils.Read_Byte(self.ser)

        nvect = zeros(N,dtype=int)
        error = zeros_like(nvect)
        uL = zeros_like(nvect)
        uR = zeros_like(nvect)

        sensor_mat = zeros((N,self.numsensors))

        self.nvect = nvect
        self.uL = uL
        self.uR = uR
        self.sensor_mat = sensor_mat
        self.error = error

        for i in range(N):
            if i > 0:
                vdiff = self.calc_v(i-1, error)
            else:
                vdiff = 0
            uL[i] = self.mysat(self.nominal_speed+vdiff)
            uR[i] = self.mysat(self.nominal_speed-vdiff)
            serial_utils.WriteByte(self.ser, 1)#new n and voltage are coming
            serial_utils.WriteInt(self.ser, i)
            serial_utils.WriteInt(self.ser, uL[i])
            serial_utils.WriteInt(self.ser, uR[i])

            nvect[i] = serial_utils.Read_Two_Bytes(self.ser)
            for j in range(self.numsensors):
                sensor_mat[i,j] = serial_utils.Read_Two_Bytes_Twos_Comp(self.ser)
            error[i] = serial_utils.Read_Two_Bytes_Twos_Comp(self.ser)
            nl_check = serial_utils.Read_Byte(self.ser)
            assert nl_check == 10, "newline problem"

        serial_utils.WriteByte(self.ser, 3)#stop test
        check_3 = serial_utils.Read_Byte(self.ser)
        print('check_3 = ' + str(check_3))
        return nvect, sensor_mat, error




class zumo_serial_ol_rotate_only(zumo_serial_connection_ol):
        def run_test(self, u):
            uL = u
            uR = -u
            return zumo_serial_connection_ol.run_test(self, uL, uR)



class zumo_serial_p_control_rotate_only(zumo_serial_connection_p_control):
    def __init__(self, ser=None, kp=0.1, \
                 **kwargs):
        zumo_serial_connection_ol.__init__(self, ser=ser, mymin=-400, \
                                           mymax=400, **kwargs)
        self.kp = kp
        self.nominal_speed = 0


class zumo_serial_connection_pd_control(zumo_serial_connection_p_control):
    def __init__(self, ser=None, kp=0.1, kd=0.1, nominal_speed=400, \
                 **kwargs):
        zumo_serial_connection_p_control.__init__(self, ser=ser, kp=kp, \
                                                  nominal_speed=nominal_speed, \
                                                  **kwargs)
        self.kd = kd


    def calc_v(self, q, error):
        ediff = error[q] - error[q-1]
        v = error[q]*self.kp + ediff*self.kd
        return v

class zumo_serial_pd_control_rotate_only(zumo_serial_connection_pd_control):
    def __init__(self, ser=None, nominal_speed=0, **kwargs):
        zumo_serial_connection_pd_control.__init__(self, **kwargs)
        self.nominal_speed = 0
        self.min = -400
        
    
## if 0:
##     t = dt*nvect

##     data = array([t, v1, v_echo]).T


##     def save_data(filename, datain):
##         #provide filename extension if there isn't one
##         fno, ext = os.path.splitext(filename)
##         if not ext:
##             ext = '.csv'
##         filename = fno + ext

##         labels = ['#t','v','theta']

##         data_str = datain.astype(str)
##         data_out = numpy.append([labels],data_str, axis=0)

##         savetxt(filename, data_out, delimiter=',')


##     serial_utils.Close_Serial(self.ser)


if __name__ == '__main__':
    #my_zumo = zumo_serial_connection_p_control(kp=0.3)
    #case = 1#OL
    #case = 2#CL
    #case = 3#CL forward motion
    #case = 4#PD forward motion
    case = 5#PD rotate only

    figure(case+100)
    clf()
    
    if case == 1:
        my_zumo = zumo_serial_ol_rotate_only()
        u = zeros(200)
        u[20:40] = 1
        u1 = zeros_like(u)
        u1[20:60] = 100.0
        u2 = zeros_like(u)
        u2[20:35] = 200.0
        u3 = zeros_like(u)
        u3[20:27] = 300.0
    elif case == 2:
        my_zumo = zumo_serial_p_control_rotate_only(kp=0.1)
    elif case == 3:
        my_zumo = zumo_serial_connection_p_control(kp=0.25)
    elif case == 4:
        my_zumo = zumo_serial_connection_pd_control(kp=0.25, kd=1)    
    elif case == 5:
        my_zumo = zumo_serial_pd_control_rotate_only(kp=0.25, kd=1)
        
    
