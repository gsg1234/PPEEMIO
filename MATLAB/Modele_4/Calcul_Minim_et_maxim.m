L = 100;
r = 50;
dmax = 200;
dmin = 100;
lt=10;

th = sym('th', 'real');

res = sin(th)-r/dmax * cos(th) + lt*sin(3*pi/4-th)/dmax-L/(dmax);
res2 = sin(th)-r/dmin * cos(th) + lt*sin(3*pi/4-th)/dmin-L/(dmin);

f_solver = matlabFunction(res, 'Vars', {th});
f_solver2 = matlabFunction(res2, 'Vars', {th});

solutionMIN=fsolve(f_solver,0)
solutionMAX=fsolve(f_solver2,0)