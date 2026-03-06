%% START OF THE ROUTINE %%
clc; clear; close all;

%% VARIABLES %%
a0=10; a1=5; a2=sqrt(2)*(a0+a1);
tita1A=-pi/3; tita1B=pi/3;

% Funciones rápidas para transformaciones
transl_m = @(px,py,pz) [1 0 0 px; 0 1 0 py; 0 0 1 pz; 0 0 0 1];
trotz_m = @(t) [cos(t) -sin(t) 0 0; sin(t) cos(t) 0 0; 0 0 1 0; 0 0 0 1];

syms x y th b1A b1B real;

%% DEFINICIÓN SIMBÓLICA %%
T1A = transl_m(a0,0,0);
T2A = trotz_m(tita1A)*transl_m(a1,0,0);
T3A = trotz_m(b1A)*transl_m(a2,0,0);
T4A = trotz_m(-pi/2-tita1A-b1A);

T1B = trotz_m(pi)*transl_m(a0,0,0);
T2B = trotz_m(tita1B)*transl_m(a1,0,0);
T3B = trotz_m(b1B)*transl_m(a2,0,0);
T4B = trotz_m(pi/2-tita1B-b1B);

Tsol = [cos(th), -sin(th), 0, x; sin(th),  cos(th), 0, y; 0, 0, 1, 0; 0, 0, 0, 1];

% Creamos el vector de residuos (Error que queremos que sea 0)
eq1 = T1A*T2A*T3A*T4A - Tsol;
eq2 = T1B*T2B*T3B*T4B - Tsol;

% 5 Ecuaciones: [PosAx, PosAy, PosBx, PosBy, OrientaciónA]
residuos_sym = [eq1(1,4); eq1(2,4); eq2(1,4); eq2(2,4); eq1(1,1)];

% CONVERSIÓN A FUNCIÓN NUMÉRICA (Crucial para velocidad)
f_num = matlabFunction(residuos_sym, 'Vars', {[x, y, th, b1A, b1B]});

%% RESOLUCIÓN %%
seed = [0, 0, 0, 0, 0]; % Tu semilla

% Usamos fsolve para encontrar la raíz exacta
options = optimoptions('fsolve', 'Display', 'iter', 'TolFun', 1e-12);
[sol_final, fval] = fsolve(@(v) f_num(v), seed, options);
sol_final

%% VISUALIZACIÓN %%
% Extraemos puntos para el Brazo A
pA0 = [0;0];
pA1 = T1A(1:2,4);
pA2 = (T1A * trotz_m(tita1A)*transl_m(a1,0,0)); pA2 = pA2(1:2,4);
pA3 = (T1A * trotz_m(tita1A)*transl_m(a1,0,0) * trotz_m(sol_final(4))*transl_m(a2,0,0)); pA3 = pA3(1:2,4);

% Extraemos puntos para el Brazo B
pB1 = (trotz_m(pi)*transl_m(a0,0,0)); pB1 = pB1(1:2,4);
pB2 = (trotz_m(pi)*transl_m(a0,0,0) * trotz_m(tita1B)*transl_m(a1,0,0)); pB2 = pB2(1:2,4);
pB3 = (pB2 + [a2*cos(pi+tita1B+sol_final(5)); a2*sin(pi+tita1B+sol_final(5))]); % Simplificado para el plot

% Gráfico
figure('Color', 'w'); hold on; grid on; axis equal;
plot([0 pA1(1) pA2(1) pA3(1)], [0 pA1(2) pA2(2) pA3(2)], '-ob', 'LineWidth', 2, 'DisplayName', 'Brazo A');
plot([0 pB1(1) pB2(1) pB3(1)], [0 pB1(2) pB2(2) pB3(2)], '-or', 'LineWidth', 2, 'DisplayName', 'Brazo B');
title('Resultado de la Cinemática');
legend;

fprintf('Error final: %.2e\n', norm(fval));