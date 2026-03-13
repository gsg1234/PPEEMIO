x = sym('x', 'real'); y = sym('y', 'real'); th1e = sym('th1e', 'real'); th3e = sym('th3e', 'real');
d1 = sym('d1', 'real'); d3=sym('d3', 'real');
th1s = sym('th1s', 'real'); th3s = sym('th3s', 'real');

T1B = transl(L,0,0);
T11 = trotz(th1s)*transl(r,0,0)*trotx(pi/2);
T12 = transl(0,0,d1);
T1T = trotx(-pi/2)*trotz(-3*pi/4)*transl(l,0,0)*trotz(-pi/2);

T3B = troty(pi)*transl(L,0,0);
T31 = trotz(th3s)*transl(r,0,0)*trotx(pi/2);
T32 = transl(0,0,d3);
T3T = trotx(-pi/2)*trotz(-3*pi/4)*transl(l,0,0)*trotz(-pi/2)*trotx(pi);

Tsol1 = [cos(th1e), -sin(th1e), 0, x; sin(th1e), cos(th1e), 0, y; 0, 0, 1, 0; 0, 0, 0, 1];
Tsol3 = [cos(th3e), -sin(th3e), 0, x; sin(th3e), cos(th3e), 0, y; 0, 0, 1, 0; 0, 0, 0, 1];
eqs = [T1B*T11*T12*T1T - Tsol1; T3B*T31*T32*T3T - Tsol3];

% Résidus pour fsolve (système sur-contraint pour assurer la coïncidence)
residuos = [eqs(1,4); eqs(2,4); eqs(5,4); eqs(6,4)];
f_solver = matlabFunction(residuos, 'Vars', {[x, y, d1, d3], th1s, th3s});
f_solverinv=matlabFunction(residuos, 'Vars',{[th1s, th3s, d1, d3], x, y});