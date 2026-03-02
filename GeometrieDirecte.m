clc; clear; close all;

L=50;
r=40;
a1=50;
a3=60;

pasos = 100; 
% Definimos rangos de movimiento (ejemplo de -pi/2 a 0)
tA_tray = linspace(-pi/2, 0, pasos);
tB_tray = linspace(-pi/2, 0, pasos);


% Crear la figura y fijar los ejes para evitar saltos visuales
figure;
axis([-100 150 -150 100 -10 100]); % Ajusta según tus dimensiones L y r
grid on; hold on;

robot1;
robot3;

tA=-pi/2;
tB=-pi/2;

%% SOLUCION %%

x = sym('x', 'real'); y = sym('y', 'real'); th = sym('th', 'real');
b1A = sym('b1A', 'real'); b1B = sym('b1B', 'real'); L1 = sym('L1', 'real');
tAs = sym('tAs', 'real'); tBs = sym('tBs', 'real');

    TBA = transl(L,0,0);
    T1A = trotz(tAs)*transl(r,0,0);
    T2A = trotz(b1A)*trotx(pi/2);
    T3A = transl(0,0,L1);
    TTA = trotx(-pi/2)*trotz(-3*pi/4)*transl(10, 0, 0)*trotz(pi/2)*transl(5,0,0);

    TBB = troty(pi)*transl(L,0,0);
    T1B = trotz(tBs)*transl(r,0,0);
    T2B = trotz(b1B)*trotx(pi/2);
    T3B = transl(0,0,a3);
    TTB = trotx(-pi/2)*trotz(-3*pi/4)*transl(10, 0, 0)*trotz(pi/2)*transl(5,0,0)*trotx(pi);

    Tsol = [cos(th), -sin(th), 0, x; sin(th), cos(th), 0, y; 0, 0, 1, 0; 0, 0, 0, 1];

    eqs = [TBA*T1A*T2A*T3A*TTA - Tsol; TBB*T1B*T2B*T3B*TTB - Tsol];
    residuos = [eqs(1,4); eqs(2,4); eqs(5,4); eqs(6,4); eqs(1,1) ; eqs(1,2) ;eqs(5,1);eqs(5,2)];
    f_solver = matlabFunction(residuos, 'Vars', {[x, y, th, b1A, b1B, L1],tAs,tBs});
    options = optimoptions('fsolve', 'Display', 'iter', 'TolFun', 1e-6);
    seed= [0, -15, -pi/2, -pi/4, -pi/4, 20];
%% PLOT %%

q1A1=zeros(pasos:1); q1A2=zeros(pasos:1); q1A3=zeros(pasos:1); 
q3A1=zeros(pasos:1); q3A2=zeros(pasos:1); q3A3=zeros(pasos:1);

%% ETAPA 1: Mover solo tA (tB fijo)
disp('Animando tA...');
for i = 1:pasos
    [q1A1(i,1:3), q3A1(i,1:3)] = calcular_paso(tA_tray(i), tB, f_solver, seed, a3);

end

%% ETAPA 2: Mover solo tB (tA fijo)
disp('Animando tB...');
tA_final = tA_tray(end);
for i = 1:pasos
    [q1A2(i,1:3), q3A2(i,1:3)] = calcular_paso(tA_final, tB_tray(i), f_solver, seed, a3);
end

%% ETAPA 3: Mover AMBOS al mismo tiempo
disp('Animando ambos...');
% Regresamos a la posición inicial
for i = 1:pasos
    [q1A3(i,1:3), q3A3(i,1:3)] = calcular_paso(tA_tray(pasos+1-i), tB_tray(pasos+1-i), f_solver, seed, a3);
end

Q1_total=[q1A1;q1A2;q1A3];
Q3_total=[q3A1;q3A2;q3A3];

R1.plot(Q1_total(1, :));
hold on;
R3.plot(Q3_total(1, :));

for k = 1:size(Q1_total, 1)
    
    % Esta es la función clave del toolbox para suavidad:
    R1.animate(Q1_total(k, :));
    R3.animate(Q3_total(k, :)); 
end



function [q1, q3] = calcular_paso(tA, tB, f_solver, seed, a3)
    options = optimoptions('fsolve', 'Display', 'none', 'TolFun', 1e-6);
    [new_sol, ~, exitflag] = fsolve(@(v) f_solver(v,tA,tB), seed , options);
    if exitflag > 0
        if new_sol(6)<0
            new_sol(6)=-new_sol(6);
            new_sol(4)=new_sol(4)+pi;
        end
        if new_sol(4)>(2*pi)
            new_sol(4)=new_sol(4)-2*pi;
        end
        q1=[tA,new_sol(4),new_sol(6)];
        q3=[tB,new_sol(5),a3];
    else
        disp("ERROR")
    end
end
