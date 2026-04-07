import sympy as sp
import numpy as np
import CodesFinales.parametres as p

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

def DH(alpha, a, d, theta):
    return trotx(alpha) * transl(a, 0, d) * trotz(theta)

n0Tn2=DH(0, 0, 0, sp.symbols('th_n'))* DH(sp.pi/2, sp.symbols('r'), sp.symbols('d_n'), 0)
n0Tne=n0Tn2*(trotx(-sp.pi/2) * trotz(-3*sp.pi/4) * transl(sp.symbols('l'), 0, 0) * trotz(-sp.pi/2))
# print("n0Tn2:")
# print(n0Tn2)
# print("n0Tne:")
# print(n0Tne)


th1_sym, th3_sym = sp.symbols('th_1 th_3', real=True)
d1_sym, d3_sym = sp.symbols('d_1 d_3', real=True)
L_sym , l_sym, r= sp.symbols('L l r', real=True)

RT10 = transl(L_sym, 0, 0)
RT30 = troty(sp.pi) * transl(L_sym, 0, 0)
M10T12=DH(0, 0, 0, th1_sym)* DH(sp.pi/2, r, d1_sym, 0)
M30T32=DH(0, 0, 0, th3_sym)* DH(sp.pi/2, r, d3_sym, 0)

RT12= RT10 * M10T12
RT32= RT30 * M30T32

print("RT12:")
print(RT12)
print("RT32:")
print(RT32)

x_diff=RT12[0, 3] - RT32[0, 3]
y_diff=RT12[1, 3] - RT32[1, 3]
print("x_diff:")
print(x_diff)
print("y_diff:")
print(y_diff)

RT1e=RT12*(trotx(-sp.pi/2) * trotz(-3*sp.pi/4) * transl(l_sym, 0, 0) * trotz(-sp.pi/2))
RT3e=RT32*(trotx(-sp.pi/2) * trotz(-3*sp.pi/4) * transl(l_sym, 0, 0) * trotz(-sp.pi/2))
print("RT1e:")
print(RT1e)
print("RT3e:")
print(RT3e) 
