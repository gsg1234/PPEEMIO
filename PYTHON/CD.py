import numpy as np
import constants as c

def get_pos_encastrement1(tita):
    return c.CENT_MOT1 + np.array([c.R*np.cos(tita), c.R*np.sin(tita)])

def get_pos_encastrement3(tita):
    return c.CENT_MOT3 + np.array([-c.R*np.cos(tita), c.R*np.sin(tita)])