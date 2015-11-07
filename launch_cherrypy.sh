#!/bin/sh

PYTHONPATH=$HOME/git/research:$HOME/git/krauss_misc:$HOME/git/bad

#:$HOME/git/report_generation:$HOME/git/teaching/:$HOME/git/pygtk_utils:$HOME/git/sympy:$HOME/git/restutils:$HOME/git/wxpython_guis:$HOME/git/pygtk_guis:$HOME/git/bad:$HOME/git/py_directive:$HOME/siue/Research/work/2013/sabbatical_summer_fall_2013/DT_TMM_gui:$HOME/git/private/:$HOME/git/personal:$HOME/git/geeknote:$HOME/git/geeknote/lib:$HOME/siue/Research/DT_TMM:$HOME/git/reinout.vanrees.org:$HOME/git/PyLit:$HOME/git/PyPDF2:/usr/local/Cellar/pil/1.1.7/lib/python2.7/site-packages/PIL:/usr/local/Cellar/python/2.7.3/Frameworks/Python.framework/Versions/2.7/lib/python2.7/site-packages/wx-2.9.4-osx_cocoa/wx/tools/
#:/usr/local/lib/python:/usr/local/lib/python2.7/site-packages/
#/Applications/Gimp.app/Contents/Resources/lib/gimp/2.0/python/
#:/Users/rkrauss/src/scipy
export PYTHONPATH
cd /home/pi/zumo_serial
sudo python get_ip.py >getiplog 2>&1
sudo python cherrypy_krauss_1.py >cherrypylog 2>&1

