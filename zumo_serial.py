from matplotlib.pyplot import *
#from scipy import *
from numpy import *
import numpy, time

import control

import time, copy, os

import serial_utils
import txt_mixin

import pdb

#righthand side for me
#portname = '/dev/tty.usbmodem1411'
#lefthand side for me

# do I want to reconnect the serial each time like this?

#time.sleep(0.1)

dt = 1.0/60#<---------- is this true for your choice of OCR1A?

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
        col_list = [self.nvect]
        if hasattr(self, 'vdiff_vect'):
            col_list.append(self.vdiff_vect)

        col_list.extend([self.uL, self.uR, \
                         self.sensor_mat, self.error])
        ## data = column_stack([self.nvect, self.uL, self.uR, \
        ##                      self.sensor_mat, self.error])
        data = column_stack(col_list)
        if hasattr(self, 'stopn'):
            if self.stopn > 0:
                data = data[0:self.stopn,:]
        data_str = data.astype('S30')
        rows, N_sense = self.sensor_mat.shape
        sen_labels = ['sensor %i' % ind for ind in range(N_sense)]
        labels = ['n']
        if hasattr(self, 'vdiff_vect'):
            labels.append('vdiff')
        labels += ['uL','uR'] + sen_labels + ['error']

        str_mat = row_stack([labels, data_str])
        txt_mixin.dump_delimited(fullname, str_mat, delim=',')
        self.data_file_name = fullname
        return str_mat
    

    def plot(self, fignum=1):
        end_ind = self.stopn
        plotn = self.nvect[0:end_ind]
        
        figure(fignum)
        clf()
        plot(plotn, self.error[0:end_ind])

        figure(fignum+1)
        clf()
        plot(plotn, self.uL[0:end_ind], plotn, self.uR[0:end_ind])

        figure(fignum+2)
        clf()
        for i in range(self.numsensors):
            plot(plotn, self.sensor_mat[:,i][0:end_ind])


        show()


    def append_plot(self, fignum, lw=2.0, label=None):
        end_ind = self.stopn
        plotn = self.nvect[0:end_ind]

        figure(fignum)
        kwargs = {'linewidth':lw}
        if label:
			kwargs['label'] = label
        plot(plotn, self.error[0:end_ind], **kwargs)



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
        post_stop_count = -1
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
                post_stop_count += 1
                print('post_stop_count = %i' % post_stop_count)
                if post_stop_count > 10:
                    break
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
            if i > 50:
                #check for completed lap
                if (not stopping) and (self.sensor_mat[i,0] > 500 and self.sensor_mat[i,-1] > 500):
                    #lap completed
                    self.stopn = i
                    t2 = time.time()
                    stopping = True
                    post_stop_count = 1
                    print('stopn = %i' % self.stopn)
                    
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
        self.total_e = abs(e_trunc).sum()
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



class zumo_serial_p_control_sys_id(zumo_serial_p_control_rotate_only):
    def save(self, basename):
        fullname = self._get_filename(basename)
        #need vdiff_vect here and in the labels
        ## data = column_stack([self.nvect, self.ref, self.uL, self.uR, \
        ##                      self.sensor_mat, self.error])
        col_list = [self.nvect, self.ref]
        if hasattr(self, 'vdiff_vect'):
            col_list.append(self.vdiff_vect)

        col_list.extend([self.uL, self.uR, \
                         self.sensor_mat, self.error])
        
        data = column_stack(col_list)
        data_str = data.astype('S30')
        rows, N_sense = self.sensor_mat.shape
        sen_labels = ['sensor %i' % ind for ind in range(N_sense)]
        labels = ['n','R (ref)']
        if hasattr(self, 'vdiff_vect'):
            labels.append('vdiff_vect')
            
        labels += ['uL','uR'] + sen_labels + ['error']

        str_mat = row_stack([labels, data_str])
        txt_mixin.dump_delimited(fullname, str_mat, delim=',')
        self.data_file_name = fullname
        return str_mat



    def calc_v(self, q):
        v = self.tracking_error[q]*self.kp
        return v


    def _init_vectors(self, N):
        zumo_serial_p_control_rotate_only._init_vectors(self, N)
        self.tracking_error = zeros(N)
        self.vdiff_vect = zeros(N)


    def calc_u(self):
        raise NotImplmentedError

        
    def run_test(self):
        serial_utils.WriteByte(self.ser, 2)#start new test
        check_2 = serial_utils.Read_Byte(self.ser)

        u = self.calc_u()
        self.ref = u

        N = len(u)
        self._init_vectors(N)

        self.stopn = -1
        stopping = False
        t1 = time.time()
        t2 = None
        for i in range(N):
            self.tracking_error[i] = self.ref[i]-self.error[i-1]
            if i > 0:
                vdiff = self.calc_v(i)#, self.tracking_error)
            else:
                vdiff = 0

            self.vdiff_vect[i] = vdiff
            
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
        self.total_e = abs(e_trunc).sum()
        return self.nvect, self.sensor_mat, self.error



    def system_id_plot(self, fignum=1):
        end_ind = self.stopn
        plotn = self.nvect[0:end_ind]
        
        figure(fignum)
        clf()
        plot(plotn, self.ref[0:end_ind], plotn, self.vdiff_vect[0:end_ind], plotn, self.error[0:end_ind])
        legend(['ref','vdiff','error'])
        
        figure(fignum+1)
        clf()
        plot(plotn, self.uL[0:end_ind], plotn, self.uR[0:end_ind])

        figure(fignum+2)
        clf()
        for i in range(self.numsensors):
            plot(plotn, self.sensor_mat[:,i][0:end_ind])


        show()



class zumo_fixed_sine(zumo_serial_p_control_sys_id):
    def __init__(self, kp=0.25, amp=500, freq=1, N=500, **kwargs):
        zumo_serial_p_control_sys_id.__init__(self, kp=kp, **kwargs)
        self.amp = amp
        self.freq = freq
        self.N = N


    def calc_u(self):
        nvect = arange(self.N)
        t = dt*nvect
        self.u = self.amp*sin(2*pi*t*self.freq)
        return self.u
    
            
    def parse_args(self, **kwargs):
        myargs = {'amp':self.amp, \
                  'N':self.N, \
                  'freq':self.freq, \
                  'Kp':self.kp,\
                  }
        myargs.update(kwargs)
        self.N = int(myargs['N'])
        self.amp = int(myargs['amp'])
        self.freq = float(myargs['freq'])
        self.kp = float(myargs['Kp'])
        self.calc_u()


    def get_report(self):
        line1 = "Fixed Sine P Control Test"
        report_lines = [line1]
        myparams = ['kp','amp','freq','N']

        for param in myparams:
            if hasattr(self, param):
                val = getattr(self, param)
                curline = '%s: %s' % (param, val)
                report_lines.append(curline)

        out = " <br> ".join(report_lines)
        return out




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
    def __init__(self, ser=None, kp=0.1, kd=0.1, ki=0.0, nominal_speed=400, N=1000,\
                 **kwargs):
        zumo_serial_connection_pd_control.__init__(self, ser=ser, kp=kp, \
                                                   kd=kd, \
                                                   nominal_speed=nominal_speed, \
                                                   **kwargs)
        self.N = N
        self.ki = ki


    def _set_float_param(self, dictin, key, attr):
        value = dictin[key]
        try:
            float_val = float(value)
        except:
            float_val = 0.0

        print('attr: %s, value: %0.4g' % (attr, float_val))
        setattr(self, attr, float_val)
            

    def parse_args(self, **kwargs):
        myargs = {'Kp':100, \
                  'Kd':20, \
                  'Ki':0, \
                  'N':1000, \
                  'min':0, \
                  'nominal_speed':400, \
                  }
        myargs.update(kwargs)
        self.N = int(myargs['N'])
        labels = ['Kp','Kd','Ki','min','nominal_speed']
        for label in labels:
            attr = label.lower()
            self._set_float_param(myargs, label, attr)



    def _init_vectors(self, N):
        zumo_serial_connection_pd_control._init_vectors(self, N)
        self.esum = zeros(N)


    def calc_score(self):
        lt = self.laptime
        te = self.total_e

        self.error_score = (500000.0-te)*50.0/300000.0
        self.laptime_score = (9.5-lt)*50.0/5.0
        self.total_score = self.error_score + self.laptime_score
        

    def get_report(self):
        self.calc_score()
        line1 = "PID Test with Forward Velocity"
        report_lines = [line1]
        myparams = ['kp','kd','ki','laptime','total_e', \
                    'error_score','laptime_score','total_score']

        labels = {'total_e':'total error', \
                  'error_score':'error score', \
                  'laptime_score':'laptime score', \
                  'total_score':'total score'}

        for param in myparams:
            if hasattr(self, param):
                val = getattr(self, param)
                if labels.has_key(param):
                    curlabel = labels[param]
                else:
                    curlabel = param
                try:
                    curline = '%s: %0.5g' % (curlabel, val)
                except:
                    curline = '%s: %s' % (curlabel, val)
                report_lines.append(curline)

        out = " <br> ".join(report_lines)
        return out


    def report_numpy(self):
        html_str = self.get_report()
        numpy_str = html_str.replace('<br>', '\n')
        print(numpy_str)

       
    def calc_v(self, q):
        self.esum[q] = self.esum[q-1] + self.error[q]
        ediff = self.error[q] - self.error[q-1]
        v = self.error[q]*self.kp + ediff*self.kd + self.ki*self.esum[q]
        return v




class zumo_serial_pid_control_rotate_only(zumo_serial_connection_pid_control):
    def __init__(self,  ser=None, kp=0.1, kd=0.1, ki=0.0, N=100, **kwargs):
        zumo_serial_connection_pid_control.__init__(self, ser=ser, kp=kp, \
                                                    ki=ki, kd=kd, **kwargs)
        self.nominal_speed = 0
        self.min = -400
        self.N = N


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



class zumo_serial_connection_pd_smc_control(zumo_serial_connection_pid_control):
    def __init__(self, ser=None, kp=0.1, kd=0.1, nominal_speed=400, \
                 **kwargs):
        zumo_serial_connection_p_control.__init__(self, ser=ser, kp=kp, \
                                                  nominal_speed=nominal_speed, \
                                                  **kwargs)
        self.kd = kd
        self.ki = 0


    def calc_v(self, q):
        ediff = self.error[q] - self.error[q-1]
        H = 1.0
        lamda = 1.0
        error_dot_noisy = ediff/dt
        cutoff = 1.0
        # Using a lowpass filter on error_dot is a good idea, but this
        # is not the right way to implement it in the time domain.
        # We need to use c2d to convert to a digital TF
        #!#low_pass_TF = (cutoff**2/((1.0j*error_dot_noisy)**2+2*0.7*cutoff*(1.0j*error_dot_noisy)+cutoff**2))
        error_dot = error_dot_noisy
        v = self.error[q]*self.kp + ediff*self.kd + H*sign(-lamda*error_dot-self.error[q])
        return v


class Digital_Compensator(object):
    def __init__(self, num, den, input_vect=None, output_vect=None):
        self.num = num
        self.den = den
        self.input = input_vect
        self.output = output_vect
        self.Nnum = len(self.num)
        self.Nden = len(self.den)


    def calc_out(self, i):
        out = 0.0
        for n, bn in enumerate(self.num):
            out += self.input[i-n]*bn

        for n in range(1, self.Nden):
            out -= self.output[i-n]*self.den[n]
        out = out/self.den[0]
        return out


class zumo_arbitrary_TF(zumo_serial_connection_pid_control):
    def __init__(self, numlist, denlist, gain=1.0, mymin=0, \
                 nominal_speed=400, **kwargs):
        zumo_serial_connection_pid_control.__init__(self, mymin=mymin, \
                                                    nominal_speed=nominal_speed, \
                                                    **kwargs)
        self.Gc = control.TransferFunction(numlist,denlist)*gain
        if hasattr(control, 'c2d'):
            c2d = control.c2d
        elif hasattr(control,'matlab'):
            c2d = control.matlab.c2d
            
        self.Gd = c2d(self.Gc, dt, 'tustin')
        self.numz = squeeze(self.Gd.num)
        self.denz = squeeze(self.Gd.den)
        self.dig_comp = Digital_Compensator(self.numz, self.denz)


    def calc_v(self, q):
        v = self.dig_comp.calc_out(q)
        self.dig_comp.output[q] = v
        return v


    def _init_vectors(self, N):
        zumo_serial_connection_pid_control._init_vectors(self, N)
        self.dig_comp_out = zeros(N)
        self.dig_comp.input = self.error
        self.dig_comp.output = self.dig_comp_out
        

    
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
    #case = 2#CL: P only; rotate only
    #case = 3#CL P only;  forward motion
    #case = 4#PD forward motion
    #case = 5#PD rotate only
    #case = 6#swept sine p control
    #case = 8
    case = 20
    
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
        N = 1000
        dt = 1.0/60
        t = arange(N)*dt
        ## T = 900*dt
        ## fmax = 3.0
        ## slope = fmax/N
        ## f = arange(0,fmax,slope)
        u = 500*sin(0.1*2*pi*t)
        figure(10)
        clf()
        plot(t,u)

        my_zumo = zumo_serial_p_control_rotate_only_swept_sine(kp=0.3)
        
        show()

    elif case == 7:
        my_zumo = zumo_serial_connection_pid_control(kp=0.25,kd=1.0,ki=0.0)
        
    
    elif case == 8:
        my_zumo = zumo_serial_connection_pd_smc_control(kp=0.25,kd=1.0,numsensors=6)
    elif case == 20:
        p = 10*2*pi
        z = p/9.0
        gain = 0.2*p/z
        my_zumo = zumo_arbitrary_TF([1,z],[1,p],gain=gain)
        
