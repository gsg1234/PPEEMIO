%% START OF THE ROUTINE %%
clc; clear;

%% VARIABLES %%
a0=10; a1=2; a2=15; a3=1;
tita1A=-90*pi/180;
tita1B=90*pi/180;

% Definimos las funciones de transformación básicas (si no las tenés en el path)
transl_m = @(x,y,z) [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
trotz_m = @(t) [cos(t) -sin(t) 0 0; sin(t) cos(t) 0 0; 0 0 1 0; 0 0 0 1];

syms x y th b1A b1B b2A b2B real; % 'real' ayuda al solver

%% DEFINICIÓN SIMBÓLICA DE LAS CADENAS %%
T1A = transl_m(a0,0,0);
T2A = trotz_m(tita1A)*transl_m(a1,0,0);
T3A = trotz_m(b1A)*transl_m(a2,0,0);
T4A = trotz_m(b2A)*transl_m(a3,0,0);
T5A = trotz_m(pi/2);

T1B = trotz_m(pi)*transl_m(a0,0,0);
T2B = trotz_m(tita1B)*transl_m(a1,0,0);
T3B = trotz_m(b1B)*transl_m(a2,0,0);
T4B = trotz_m(b2B)*transl_m(a3,0,0);
T5B = trotz_m(-pi/2);

Tsol = [cos(th), -sin(th), 0, x;
        sin(th),  cos(th), 0, y;
        0, 0, 1, 0;
        0, 0, 0, 1];

% Diferencias (queremos que sean cero)
res1 = T1A*T2A*T3A*T4A*T5A - Tsol;
res2 = T1B*T2B*T3B*T4B*T5B - Tsol;
res3 = b1A - b1B + b2A - b2B + tita1A - tita1B + 2*pi;

% Vector de residuos (7 ecuaciones para 7 incógnitas)
% Seleccionamos los elementos relevantes (x, y, y componentes de rotación)
residuos_sym = [res1(1,4); res1(2,4); res2(1,4); res2(2,4); res1(1,1); res2(2,2); res3];

% ¡PASO CLAVE!: Convertir a función numérica rápida
% El orden de entrada será: [x, y, th, b1A, b1B, b2A, b2B]
f_num = matlabFunction(residuos_sym, 'Vars', {[x, y, th, b1A, b1B, b2A, b2B]});

%% CONFIGURACIÓN DEL ENJAMBRE (PSO) %%
nvars = 7;
% Límites: [x, y, th, b1A, b1B, b2A, b2B]
% Ajustá estos límites según el espacio de trabajo de tu robot
lb = [-30, -30, -pi, -pi/2, 0, -pi/2, 0]; 
ub = [ 30,  30,  0,  0,  pi/2,  0,  pi/2];

% Función de costo para PSO (escalar: suma de cuadrados de los residuos)
objetivo_pso = @(v) sum(f_num(v).^2);

fprintf('Iniciando búsqueda global con PSO...\n');
options_pso = optimoptions('particleswarm', ...
    'SwarmSize', 100, ... % Más partículas para 7 variables
    'MaxIterations', 200, ...
    'Display', 'iter');

[x_global, fval_pso] = particleswarm(objetivo_pso, nvars, lb, ub, options_pso);

%% REFINAMIENTO LOCAL (PULIDO) %%
fprintf('\nRefinando solución con fsolve...\n');
options_fsolve = optimoptions('fsolve', 'Display', 'iter', 'TolFun', 1e-12);

% fsolve necesita el vector de residuos, no el escalar
[sol_final, fval_final, exitflag] = fsolve(@(v) f_num(v), x_global, options_fsolve);

%% RESULTADOS %%
vars_nombres = {'x', 'y', 'th', 'b1A', 'b1B', 'b2A', 'b2B'};
disp('--------------------------------------');
disp('Solución Final Encontrada:');
for i = 1:7
    fprintf('%s = %.6f\n', vars_nombres{i}, sol_final(i));
end
fprintf('Error residual total (norma): %.2e\n', norm(f_num(sol_final)));
disp('--------------------------------------');