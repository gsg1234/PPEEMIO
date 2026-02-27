%% START OF THE ROUTINE %%
clc; clear; close all;

%% VARIABLES %%
a0=10; a1=5; a2=sqrt(2)*(a0+a1);
tita1A=-1; tita1B=0.9;

% Funciones auxiliares
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

residuos_sym = [T1A*T2A*T3A*T4A - Tsol; T1B*T2B*T3B*T4B - Tsol];
% Extraemos solo 5 ecuaciones clave para resolver
eqs = [residuos_sym(1,4); residuos_sym(2,4); residuos_sym(5,4); residuos_sym(6,4); residuos_sym(1,1)];
f_num = matlabFunction(eqs, 'Vars', {[x, y, th, b1A, b1B]});

%% SOLUCIÓN NUMÉRICA (PSO + FSOLVE) %%
lb = [-(a0+a1), -(a0+a1), -pi, -pi, 0]; ub = [a0+a1, 0, 0, 0, pi];
options_pso = optimoptions('particleswarm', 'SwarmSize', 50, 'Display', 'none');
x_global = particleswarm(@(v) sum(f_num(v).^2), 5, lb, ub, options_pso);

options_fsolve = optimoptions('fsolve', 'Display', 'none', 'TolFun', 1e-12);
sol = fsolve(@(v) f_num(v), x_global, options_fsolve);

%% EXTRACCIÓN DE COORDENADAS PARA GRAFICAR %%
% Asignamos los valores hallados
x_val=sol(1); y_val=sol(2); th_val=sol(3); b1A_val=sol(4); b1B_val=sol(5);

% Calculamos matrices numéricas de cada articulación
% Brazo A
A0 = eye(4);
A1 = T1A;
A2 = A1 * (trotz_m(tita1A)*transl_m(a1,0,0));
A3 = A2 * (trotz_m(b1A_val)*transl_m(a2,0,0));
A4 = A3 * (trotz_m(-pi/2-tita1A-b1A_val)); % End Effector A

% Brazo B
B0 = eye(4);
B1 = trotz_m(pi)*transl_m(a0,0,0);
B2 = B1 * (trotz_m(tita1B)*transl_m(a1,0,0));
B3 = B2 * (trotz_m(b1B_val)*transl_m(a2,0,0));
B4 = B3 * (trotz_m(pi/2-tita1B-b1B_val)); % End Effector B

% Juntamos puntos (x, y)
pA = [A0(1:2,4), A1(1:2,4), A2(1:2,4), A3(1:2,4), A4(1:2,4)];
pB = [B0(1:2,4), B1(1:2,4), B2(1:2,4), B3(1:2,4), B4(1:2,4)];

%% PLOT %%
figure('Color', 'w', 'Name', 'Cinemática del Robot');
hold on; grid on; axis equal;

% Dibujar Brazo A (Azul)
plot(pA(1,:), pA(2,:), '-o', 'LineWidth', 3, 'MarkerSize', 6, 'Color', [0 0.447 0.741], 'DisplayName', 'Brazo A');
% Dibujar Brazo B (Rojo)
plot(pB(1,:), pB(2,:), '-o', 'LineWidth', 3, 'MarkerSize', 6, 'Color', [0.85 0.32 0.09], 'DisplayName', 'Brazo B');

% Dibujar el Efector Final (la plataforma que une ambos)
plot([pA(1,5) pB(1,5)], [pA(2,5) pB(2,5)], 'k--', 'LineWidth', 2, 'DisplayName', 'Plataforma');

% Estética
xlabel('X [unidades]'); ylabel('Y [unidades]');
title(['Configuración Hallada (Error: ', num2str(norm(f_num(sol)), '%.2e'), ')']);
legend('Location', 'northeastoutside');
xlim([-40 40]); ylim([-40 40]);

fprintf('Solución: x=%.2f, y=%.2f, th=%.2f rad\n', x_val, y_val, th_val);