from matplotlib.pyplot import *
#from scipy import *
from numpy import *
import numpy, time

#import control

import time, copy, os

import serial_utils
import txt_mixin

import pdb

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


    def _import_ser(self, force=False):
        if (self.ser is None) or force:
            from myserial import ser
            self.ser = ser
            self.ser_check_count = 0
            

    def open_and_check_serial(self):
        self._import_ser()
        self.ser_check_count += 1
        serial_utils.WriteByte(self.ser, 0)
        time.sleep(0.1)
        if self.ser.inWaiting():
            debug_line = serial_utils.Read_Line(self.ser)
            line_str = ''.join(debug_line)
            return line_str
        else:
            return None
            

    def open_serial(self):
        # check if it is open first
        from myserial import ser

        time.sleep(3.0)# <--- will this work under windows?
                       # does this work better with a Z32U4
                       # if it isn't rebooting?
        # could I put this in a while loop rather than the hard 3.0 sleep?
        # - if you don't get a response, the Uno is ready yet
        # - I could set up another Uno serial case that responds with just one byte
        #   to acknowledge wake up
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
        

    def run_test(self, uL=None, uR=None):
        if uL is None:
            uL = self.uL
        if uR is None:
            uR = self.uR
        serial_utils.WriteByte(self.ser, 2)#start new test
        check_2 = serial_utils.Read_Byte(self.ser)

        N = len(uL)
        self.stopn = N
        nvect = zeros(N,dtype=int)
        numsensors = self.numsensors
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
        self.nvect = nvect
        self.sensor_mat = sensor_mat
        self.error = error
        return nvect, sensor_mat, error


    def _get_filename(self, basename):
        #fullname = 'ol_%s.csv' % basename
        fullname = basename + '.csv'
        return fullname


    def save(self, basename):
        fullname = self._get_filename(basename)
        data = column_stack([self.nvect, self.uL, self.uR, \
                             self.sensor_mat, self.error])
        data_str = data.astype('S30')
        rows, N_sense = self.sensor_mat.shape
        sen_labels = ['sensor %i' % ind for ind in range(N_sense)]
        labels = ['n','uL','uR'] + sen_labels + ['error']

        str_mat = row_stack([labels, data_str])
        txt_mixin.dump_delimited(fullname, str_mat, delim=',')
        self.data_file_name = fullname
        return str_mat
    

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


    def append_plot(self, fignum, lw=2.0, label=None):
        figure(fignum)
        kwargs = {'linewidth':lw}
        if label:
			kwargs['label'] = label
        plot(self.nvect, self.error, **kwargs)



class zumo_serial_connection_p_control(zumo_serial_connection_ol):
    def __init__(self, ser=None, kp=0.1, nominal_speed=400, \
                 **kwargs):
        zumo_serial_connection_ol.__init__(self, ser=ser, **kwargs)
        self.kp = kp
        self.nominal_speed = nominal_speed
        
    ## def _get_filename(self, basename):
    ##     fullname = 'p_control_%s_kp=%0.4g.csv' % (basename, self.kp)
    ##     return fullname

    def calc_v(self, q):
        v = self.error[q]*self.kp
        return v


    def _init_vectors(self, N):
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

    

    def run_test(self, N=None):
        if N is None:
            if hasattr(self, 'N'):
                N = int(self.N)
            else:
                N = 500
        serial_utils.WriteByte(self.ser, 2)#start new test
        check_2 = serial_utils.Read_Byte(self.ser)

        self._init_vectors(N)
        
        self.stopn = -1
        stopping = False
        t1 = time.time()
        t2 = None
        for i in range(N):
            if i > 0:
                vdiff = self.calc_v(i-1)
            else:
                vdiff = 0

            if stopping:
                self.uL[i] = 0
                self.uR[i] = 0
            else:
                self.uL[i] = self.mysat(self.nominal_speed+vdiff)
                self.uR[i] = self.mysat(self.nominal_speed-vdiff)

            # do I organize this into sub-methods and actually stop the test
            # if we are back to the finish line, or do I just sit there
            # sending 0's for speed and reading the same stopped data?
            serial_utils.WriteByte(self.ser, 1)#new n and voltage are coming
            serial_utils.WriteInt(self.ser, i)
            serial_utils.WriteInt(self.ser, self.uL[i])
            serial_utils.WriteInt(self.ser, self.uR[i])

            self.nvect[i] = serial_utils.Read_Two_Bytes(self.ser)
            for j in range(self.numsensors):
                self.sensor_mat[i,j] = serial_utils.Read_Two_Bytes_Twos_Comp(self.ser)
            if i > 100:
                #check for completed lap
                if self.sensor_mat[i,0] > 500 and self.sensor_mat[i,-1] > 500:
                    #lap completed
                    self.stopn = i
                    t2 = time.time()
                    stopping = True
                    
            self.error[i] = serial_utils.Read_Two_Bytes_Twos_Comp(self.ser)
            nl_check = serial_utils.Read_Byte(self.ser)
            assert nl_check == 10, "newline problem"

        serial_utils.WriteByte(self.ser, 3)#stop test
        check_3 = serial_utils.Read_Byte(self.ser)
        print('check_3 = ' + str(check_3))

        if t2 is not None:
            self.laptime = t2-t1
        else:
            self.laptime = 999.999
        e_trunc = self.error[0:self.stopn]
        self.total_e = e_trunc.sum()
        return self.nvect, self.sensor_mat, self.error




class zumo_serial_ol_rotate_only(zumo_serial_connection_ol):
    def parse_args(self, **kwargs):
        myargs = {'amp':100, \
                  'width':20, \
                  'N':200, \
                  'start':10, \
                  }
        myargs.update(kwargs)
        self.N = int(myargs['N'])
        self.start = int(myargs['start'])
        self.u = zeros(self.N)
        self.width = int(myargs['width'])
        self.stop = self.start+self.width
        self.amp = int(myargs['amp'])
        self.u[self.start:self.stop] = self.amp


    def get_report(self):
        line1 = "OL Rotate Only Test"
        report_lines = [line1]
        myparams = ['amp','width','N']

        for param in myparams:
            if hasattr(self, param):
                val = getattr(self, param)
                curline = '%s: %s' % (param, val)
                report_lines.append(curline)
                
        out = " <br> ".join(report_lines)
        return out
    
        
    def run_test(self, u=None):
        if u is None:
            u = self.u
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



class zumo_serial_p_control_rotate_only_swept_sine(zumo_serial_p_control_rotate_only):
    def save(self, basename):
        fullname = self._get_filename(basename)
        data = column_stack([self.nvect, self.ref, self.uL, self.uR, \
                             self.sensor_mat, self.error])
        data_str = data.astype('S30')
        rows, N_sense = self.sensor_mat.shape
        sen_labels = ['sensor %i' % ind for ind in range(N_sense)]
        labels = ['n','R (ref)','uL','uR'] + sen_labels + ['error']

        str_mat = row_stack([labels, data_str])
        txt_mixin.dump_delimited(fullname, str_mat, delim=',')
        self.data_file_name = fullname
        return str_mat


    def calc_v(self, q):
        v = self.error[q]*self.kp
        return v


    def _init_vectors(self, N):
        zumo_serial_p_control_rotate_only._init_vectors(self, N)
        self.tracking_error = zeros(N)
        
    def run_test(self, u):
        serial_utils.WriteByte(self.ser, 2)#start new test
        check_2 = serial_utils.Read_Byte(self.ser)
        N = len(u)
        self._init_vectors(N)

        self.ref = u

        self.stopn = -1
        stopping = False
        t1 = time.time()
        t2 = None
        for i in range(N):
            self.tracking_error[i] = self.ref[i]-self.error[i-1]
            if i > 0:
                vdiff = self.calc_v(i, self.tracking_error)
            else:
                vdiff = 0

            if stopping:
                self.uL[i] = 0
                self.uR[i] = 0
            else:
                self.uL[i] = self.mysat(self.nominal_speed-vdiff)
                self.uR[i] = self.mysat(self.nominal_speed+vdiff)

            # do I organize this into sub-methods and actually stop the test
            # if we are back to the finish line, or do I just sit there
            # sending 0's for speed and reading the same stopped data?
            serial_utils.WriteByte(self.ser, 1)#new n and voltage are coming
            serial_utils.WriteInt(self.ser, i)
            serial_utils.WriteInt(self.ser, self.uL[i])
            serial_utils.WriteInt(self.ser, self.uR[i])

            self.nvect[i] = serial_utils.Read_Two_Bytes(self.ser)
            for j in range(self.numsensors):
                self.sensor_mat[i,j] = serial_utils.Read_Two_Bytes_Twos_Comp(self.ser)
            ## if i > 100:
            ##     #check for completed lap
            ##     if sensor_mat[i,0] > 500 and sensor_mat[i,-1] > 500:
            ##         #lap completed
            ##         self.stopn = i
            ##         t2 = time.time()
            ##         stopping = True

            self.error[i] = serial_utils.Read_Two_Bytes_Twos_Comp(self.ser)
            nl_check = serial_utils.Read_Byte(self.ser)
            assert nl_check == 10, "newline problem"

        serial_utils.WriteByte(self.ser, 3)#stop test
        check_3 = serial_utils.Read_Byte(self.ser)
        print('check_3 = ' + str(check_3))
        self.stopn = N
        self.laptime = 999.999
        e_trunc = self.error[0:self.stopn]
        self.total_e = e_trunc.sum()
        return self.nvect, self.sensor_mat, self.error





class zumo_serial_connection_pd_control(zumo_serial_connection_p_control):
    def __init__(self, ser=None, kp=0.1, kd=0.1, nominal_speed=400, \
                 **kwargs):
        zumo_serial_connection_p_control.__init__(self, ser=ser, kp=kp, \
                                                  nominal_speed=nominal_speed, \
                                                  **kwargs)
        self.kd = kd
        self.ki = 0


    def calc_v(self, q):
        ediff = self.error[q] - self.error[q-1]
        v = self.error[q]*self.kp + ediff*self.kd
        return v


class zumo_serial_pd_control_rotate_only(zumo_serial_connection_pd_control):
    def __init__(self, ser=None, nominal_speed=0, **kwargs):
        zumo_serial_connection_pd_control.__init__(self, **kwargs)
        self.nominal_speed = 0
        self.min = -400


    def parse_args(self, **kwargs):
        myargs = {'Kp':100, \
                  'Kd':20, \
                  'N':300, \
                  }
        myargs.update(kwargs)
        self.N = int(myargs['N'])
        self.kp = float(myargs['Kp'])
        self.kd = float(myargs['Kd'])


    def get_report(self):
        line1 = "PD Rotate Only Test"
        report_lines = [line1]
        myparams = ['kp','kd']

        for param in myparams:
            if hasattr(self, param):
                val = getattr(self, param)
                curline = '%s: %s' % (param, val)
                report_lines.append(curline)

        out = " <br> ".join(report_lines)
        return out


class zumo_serial_connection_pid_control(zumo_serial_connection_pd_control):
    def __init__(self, ser=None, kp=0.1, kd=0.1, ki=0.0, nominal_speed=400, \
                 **kwargs):
        zumo_serial_connection_pd_control.__init__(self, ser=ser, kp=kp, \
                                                   kd=kd, \
                                                   nominal_speed=nominal_speed, \
                                                   **kwargs)
        self.ki = ki


    def parse_args(self, **kwargs):
        myargs = {'Kp':100, \
                  'Kd':20, \
                  'Ki':0, \
                  'N':300, \
                  }
        myargs.update(kwargs)
        self.N = int(myargs['N'])
        self.kp = float(myargs['Kp'])
        self.kd = float(myargs['Kd'])
        self.ki = float(myargs['Ki'])



    def _init_vectors(self, N):
        zumo_serial_connection_pd_control._init_vectors(self, N)
        self.esum = zeros(N)

        
    def calc_v(self, q):
        self.esum[q] = self.esum[q-1] + self.error[q]
        ediff = self.error[q] - self.error[q-1]
        v = self.error[q]*self.kp + ediff*self.kd + self.ki*self.esum[q]
        return v




class zumo_serial_pid_control_rotate_only(zumo_serial_connection_pid_control):
    def __init__(self,  ser=None, kp=0.1, kd=0.1, ki=0.0, **kwargs):
        zumo_serial_connection_pid_control.__init__(self, ser=ser, kp=kp, \
                                                    ki=ki, kd=kd, **kwargs)
        self.nominal_speed = 0
        self.min = -400


    ## def parse_args(self, **kwargs):
    ##     myargs = {'Kp':100, \
    ##               'Kd':20, \
    ##               'Ki':0, \
    ##               'N':300, \
    ##               }
    ##     myargs.update(kwargs)
    ##     self.N = int(myargs['N'])
    ##     self.kp = float(myargs['Kp'])
    ##     self.kd = float(myargs['Kd'])
    ##     self.ki = float(myargs['Ki'])


    def get_report(self):
        line1 = "PID Rotate Only Test"
        report_lines = [line1]
        myparams = ['kp','kd','ki']

        for param in myparams:
            if hasattr(self, param):
                val = getattr(self, param)
                curline = '%s: %s' % (param, val)
                report_lines.append(curline)

        out = " <br> ".join(report_lines)
        return out

    
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
    case = 2#CL: P only; rotate only
    #case = 3#CL P only;  forward motion
    #case = 4#PD forward motion
    #case = 5#PD rotate only
    #case = 6#swept sine p control
    
    figure(case+100)
    clf()
    
    if case == 1:
        my_zumo = zumo_serial_ol_rotate_only()
        u = zeros(200)
        u[20:40] = 1
        u1 = zeros_like(u)
        u1[20:70] = -100.0
        u2 = zeros_like(u)
        u2[20:35] = -200.0
        u3 = zeros_like(u)
        u3[20:27] = -300.0
    elif case == 2:
        my_zumo = zumo_serial_p_control_rotate_only(kp=0.1)
    elif case == 3:
        my_zumo = zumo_serial_connection_p_control(kp=0.25)
    elif case == 4:
        my_zumo = zumo_serial_connection_pd_control(kp=0.25, kd=1, numsensors=6)    
    elif case == 5:
        my_zumo = zumo_serial_pd_control_rotate_only(kp=0.25, kd=1)
    elif case == 6:
        N = 2000
        dt = 0.01
        t = arange(N)*dt
        T = 900*dt
        fmax = 3.0
        slope = fmax/N
        f = arange(0,fmax,slope)
        u = 500*sin(2*pi*f*t)
        figure(10)
        clf()
        plot(t,u)

        my_zumo = zumo_serial_p_control_rotate_only_swept_sine(kp=0.3)
        
        show()
        
    
