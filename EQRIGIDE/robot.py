import sympy as sp
import numpy as np
import parametres as p

#Matrices de rotation
def transl(x, y, z):
    return sp.Matrix([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

def trotx(angle):
    c, s = sp.cos(angle), sp.sin(angle)
    return sp.Matrix([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def troty(angle):
    c, s = sp.cos(angle), sp.sin(angle)
    return sp.Matrix([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

def trotz(angle):
    c, s = sp.cos(angle), sp.sin(angle)
    return sp.Matrix([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

x_sym, y_sym, d1_sym, d3_sym = sp.symbols('x y d1 d3', real=True)
th1s, th3s = sp.symbols('th1s th3s', real=True)

T1B = transl(p.L, 0, 0)
T11 = trotz(th1s) * transl(p.r, 0, 0) * trotx(sp.pi/2)
T12 = transl(0, 0, d1_sym)
T1T = trotx(-sp.pi/2) * trotz(-3*sp.pi/4) * transl(p.l_len, 0, 0) * trotz(-sp.pi/2)
T_R1 = T1B * T11 * T12 * T1T

T3B = troty(sp.pi) * transl(p.L, 0, 0)
T31 = trotz(th3s) * transl(p.r, 0, 0) * trotx(sp.pi/2)
T32 = transl(0, 0, d3_sym)
T3T = trotx(-sp.pi/2) * trotz(-3*sp.pi/4) * transl(p.l_len, 0, 0) * trotz(-sp.pi/2) * trotx(sp.pi)
T_R3 = T3B * T31 * T32 * T3T

residuos = sp.Matrix([
    T_R1[0, 3] - x_sym, T_R1[1, 3] - y_sym,
    T_R3[0, 3] - x_sym, T_R3[1, 3] - y_sym
])

f_sym_fast = sp.lambdify((x_sym, y_sym, d1_sym, d3_sym, th1s, th3s), residuos, 'numpy')

def f_solver(v, th1, th3):
    return f_sym_fast(v[0], v[1], v[2], v[3], th1, th3).flatten()

def f_solverinv(v, x_val, y_val):
    return f_sym_fast(x_val, y_val, v[2], v[3], v[0], v[1]).flatten()