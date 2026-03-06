clc; clear; close all;



%% Paramètres Géométriques
L = 50;
r = 40;
a1 = 50;
a3 = 60;
pasos = 100; 

% Vecteurs de mouvement (Trajectoires)
tA_tray = linspace(-pi/2, 0, pasos);
tB_tray = linspace(-pi/2, 0, pasos);


% Initialisation des robots (Scripts externes)
robot0;
robot2;

tA = -pi/2;
tB = -pi/2;

%% RÉSOLUTION SYMBOLIQUE %%
z = sym('z', 'real'); y = sym('y', 'real'); th = sym('th', 'real');
b1A = sym('b1A', 'real'); b1B = sym('b1B', 'real'); L1 = sym('L1', 'real');
tAs = sym('tAs', 'real'); tBs = sym('tBs', 'real');

TBA = troty(-pi/2)*transl(L,0,0);
T1A = trotz(tAs)*transl(r,0,0);
T2A = trotz(b1A)*trotx(pi/2);
T3A = transl(0,0,L1);
TTA = trotx(-pi/2)*trotz(-3*pi/4)*transl(10, 0, 0)*trotz(pi/2)*transl(5,0,0)*trotx(-pi/2);

TBB = troty(pi/2)*transl(L,0,0);
T1B = trotz(tBs)*transl(r,0,0);
T2B = trotz(b1B)*trotx(pi/2);
T3B = transl(0,0,a3);
TTB = trotx(-pi/2)*trotz(-3*pi/4)*transl(10, 0, 0)*trotz(pi/2)*transl(5,0,0)*trotx(pi/2);

Tsol = [1,0,0,0; 0, cos(th), -sin(th), y; 0, sin(th), cos(th), z; 0, 0, 0, 1]*trotz(-pi/2);
eqs = [TBA*T1A*T2A*T3A*TTA - Tsol; TBB*T1B*T2B*T3B*TTB - Tsol];

% Résidus pour fsolve (système sur-contraint pour assurer la coïncidence)
residuos = [eqs(2,4); eqs(3,4); eqs(6,4); eqs(7,4); eqs(2,1); eqs(6,1); eqs(2,3); eqs(6,3)];
f_solver = matlabFunction(residuos, 'Vars', {[z, y, th, b1A, b1B, L1], tAs, tBs});
% Valeurs initiales (Seed)
seed = [0, -20, -pi/2, pi/4, pi/4, 30];

%% CALCUL DES TRAJECTOIRES %%
q1A1=zeros(2*pasos,3); q1A2=zeros(2*pasos,3); q1A3=zeros(2*pasos,3); 
q3A1=zeros(2*pasos,3); q3A2=zeros(2*pasos,3); q3A3=zeros(2*pasos,3);

% ÉTAPE 1 : Mouvement de tA uniquement
disp('Calcul Étape 1 : tA en mouvement...');
for i = 1:pasos
    [q1A1(i,:), q3A1(i,:),seed] = calculer_pas(tA_tray(i), tB, f_solver, seed, a3);
    [q1A1(i+pasos,:), q3A1(i+pasos,:),seed] = calculer_pas(tA_tray(pasos+1-i), tB, f_solver, seed, a3);
end

% ÉTAPE 2 : Mouvement de tB uniquement
disp('Calcul Étape 2 : tB en mouvement...');
for i = 1:pasos
    [q1A2(i,:), q3A2(i,:),seed] = calculer_pas(tA, tB_tray(i), f_solver, seed, a3);
    [q1A2(i+pasos,:), q3A2(i+pasos,:),seed] = calculer_pas(tA, tB_tray(pasos+1-i), f_solver, seed, a3);
end

% ÉTAPE 3 : Mouvement simultané
disp('Calcul Étape 3 : tA et tB en mouvement...');
for i = 1:pasos
    [q1A3(i,:), q3A3(i,:),seed] = calculer_pas(tA_tray(i), tB_tray(i), f_solver, seed, a3);
    [q1A3(i+pasos,:), q3A3(i+pasos,:),seed] = calculer_pas(tA_tray(pasos+1-i), tB_tray(pasos+1-i), f_solver, seed, a3);
end

% Fusion des trajectoires
Q0_total = [q1A1; q1A2; q1A3];
Q2_total = [q3A1; q3A2; q3A3];

%% ANIMATION %%
figure('Name', 'Animation Robots ENIB');
R0.plot(Q0_total(1,:), 'delay', 0, 'nojaxes', 'noshadow', 'workspace', [-10 100 -150 100 -150 100]);
hold on;
R2.plot(Q2_total(1,:), 'delay', 0, 'nojaxes', 'noshadow', 'workspace', [-10 100 -150 100 -150 100]);
view([90 0]);
camroll(90);
axis equal;
axis manual;
grid on;

disp('Lancement de l''animation...');
for k = 1:size(Q0_total, 1)
    R0.animate(Q0_total(k, :));
    R2.animate(Q2_total(k, :));
    drawnow limitrate; % Optimisation de la vitesse d'affichage
    pause(0.01);
end
hold off;

%% FONCTION AUXILIAIRE %%
function [q0, q2, new_seed] = calculer_pas(tA, tB, f_solver, seed, a3)
    % lsqnonlin minimizes the vector returned by f_solver
    
    lb = [-100, -200, -pi, -pi, -pi, 0];   % L1 cannot be negative
    ub = [ 100,  0,  pi,  pi,  pi, 150]; % L1 max length 150
    
    
    options = optimoptions('lsqnonlin', 'Display', 'off');
    options.Algorithm = 'trust-region-reflective';
    [new_sol, resnorm, ~, exitflag] = lsqnonlin(@(v) f_solver(v, tA, tB), seed, lb, ub, options);
    if exitflag > 0
        % Handle physical consistency for L1
        if new_sol(6) < 0
            new_sol(6) = abs(new_sol(6));
            new_sol(4) = mod(new_sol(4) + pi, 2*pi);
        end
        
        q0 = [tA, new_sol(4), new_sol(6)];
        q2 = [tB, new_sol(5), a3];
        new_seed = new_sol; % Pass the solution back to be the next seed
    else
        warning('Convergence failed at tA=%f. Keeping previous seed.', tA);
        q0 = [tA, 0, 0]; 
        q2 = [tB, 0, a3];
        new_seed = seed; % Keep old seed to try and "recover" in next step
    end
end