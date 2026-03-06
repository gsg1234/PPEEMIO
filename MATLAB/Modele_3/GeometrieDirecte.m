clc; clear; close all;

% Désactiver l'avertissement du système non carré (pour fsolve)
warning('off', 'optim:rootfind:SystemNotSquare');

%% Paramètres Géométriques
L = 100;
r = 50;
d1 = 200;
d3 = 200;
pasos = 100; 

% Vecteurs de mouvement (Trajectoires)
tA_tray = linspace(-pi/4, 0, pasos);
tB_tray = linspace(-pi/4, 0, pasos);


% Initialisation des robots (Scripts externes)
robot1;
robot3;

thA = -pi/4;
thB = -pi/4;

%% RÉSOLUTION SYMBOLIQUE %%
x = sym('x', 'real'); y = sym('y', 'real'); th = sym('th', 'real');
b1A = sym('b1A', 'real'); b1B = sym('b1B', 'real'); d1 = sym('d1', 'real');
thAs = sym('thAs', 'real'); thBs = sym('thBs', 'real');

TBA = transl(L,0,0);
T1A = trotz(thAs)*transl(r,0,0);
T2A = trotz(b1A)*trotx(pi/2);
T3A = transl(0,0,d1);
TTA = trotx(-pi/2)*trotz(-3*pi/4)*transl(10, 0, 0)*trotz(pi/2)*transl(5,0,0);

TBB = troty(pi)*transl(L,0,0);
T1B = trotz(thBs)*transl(r,0,0);
T2B = trotz(b1B)*trotx(pi/2);
T3B = transl(0,0,d3);
TTB = trotx(-pi/2)*trotz(-3*pi/4)*transl(10, 0, 0)*trotz(pi/2)*transl(5,0,0)*trotx(pi);

Tsol = [cos(th), -sin(th), 0, x; sin(th), cos(th), 0, y; 0, 0, 1, 0; 0, 0, 0, 1];
eqs = [TBA*T1A*T2A*T3A*TTA - Tsol; TBB*T1B*T2B*T3B*TTB - Tsol];

% Résidus pour fsolve (système sur-contraint pour assurer la coïncidence)
residuos = [eqs(1,4); eqs(2,4); eqs(5,4); eqs(6,4); eqs(1,1); eqs(1,2); eqs(5,1); eqs(5,2)];
f_solver = matlabFunction(residuos, 'Vars', {[x, y, th, b1A, b1B, d1], thAs, thBs});

% Valeurs initiales (Seed)
seed = [0, -150, -pi/2, -pi/2, -pi/2, 100];

%% CALCUL DES TRAJECTOIRES %%
q1A1=zeros(2*pasos,3); q1A2=zeros(2*pasos,3); q1A3=zeros(2*pasos,3); 
q3A1=zeros(2*pasos,3); q3A2=zeros(2*pasos,3); q3A3=zeros(2*pasos,3);

% ÉTAPE 1 : Mouvement de thA uniquement
disp('Calcul Étape 1 : thA en mouvement...');
for i = 1:pasos
    [q1A1(i,:), q3A1(i,:),seed] = calculer_pas(tA_tray(i), thB, f_solver, seed, d3);
    [q1A1(i+pasos,:), q3A1(i+pasos,:),seed] = calculer_pas(tA_tray(pasos+1-i), thB, f_solver, seed, d3);
end

% ÉTAPE 2 : Mouvement de thB uniquement
disp('Calcul Étape 2 : thB en mouvement...');
for i = 1:pasos
    [q1A2(i,:), q3A2(i,:),seed] = calculer_pas(thA, tB_tray(i), f_solver, seed, d3);
    [q1A2(i+pasos,:), q3A2(i+pasos,:),seed] = calculer_pas(thA, tB_tray(pasos+1-i), f_solver, seed, d3);
end

% ÉTAPE 3 : Mouvement simultané
disp('Calcul Étape 3 : thA et thB en mouvement...');
for i = 1:pasos
    [q1A3(i,:), q3A3(i,:),seed] = calculer_pas(tA_tray(i), tB_tray(i), f_solver, seed, d3);
    [q1A3(i+pasos,:), q3A3(i+pasos,:),seed] = calculer_pas(tA_tray(pasos+1-i), tB_tray(pasos+1-i), f_solver, seed, d3);
end

% Fusion des trajectoires
Q1_total = [q1A1; q1A2; q1A3];
Q3_total = [q3A1; q3A2; q3A3];

%% ANIMATION %%
figure('Name', 'Animation Robots ENIB');
R1.plot3d(Q1_total(1,:), 'delay', 0, 'nojaxes', 'noshadow', 'workspace', [-200 200 -300 100 -10 100]);
hold on;
R3.plot3d(Q3_total(1,:), 'delay', 0, 'nojaxes', 'noshadow', 'workspace', [-200 200 -300 100 -10 100]);
view(2);
axis equal;
axis manual;
grid on;

disp('Lancement de l''animation...');
for k = 1:size(Q1_total, 1)
    R1.animate(Q1_total(k, :));
    R3.animate(Q3_total(k, :));
    drawnow limitrate; % Optimisation de la vitesse d'affichage
    pause(0.01);
end
hold off;

%% FONCTION AUXILIAIRE %%
function [q1, q3, new_seed] = calculer_pas(thA, thB, f_solver, seed, d3)
    % lsqnonlin minimizes the vector returned by f_solver
    
    lb = [-300, -300, -pi, -pi, -pi, 0];   % d1 cannot be negative
    ub = [ 300,  0,  pi,  pi,  pi, 300]; % d1 max length 150
    
    
    options = optimoptions('lsqnonlin', 'Display', 'off');
    options.Algorithm = 'trust-region-reflective';
    [new_sol, resnorm, ~, exitflag] = lsqnonlin(@(v) f_solver(v, thA, thB), seed, lb, ub, options);
    if exitflag > 0
        % Handle physical consistency for d1
        if new_sol(6) < 0
            new_sol(6) = abs(new_sol(6));
            new_sol(4) = mod(new_sol(4) + pi, 2*pi);
        end
        
        q1 = [thA, new_sol(4), new_sol(6)];
        q3 = [thB, new_sol(5), d3];
        new_seed = new_sol; % Pass the solution back to be the next seed
    else
        warning('Convergence failed at thA=%f. Keeping previous seed.', thA);
        q1 = [thA, 0, 0]; 
        q3 = [thB, 0, d3];
        new_seed = seed; % Keep old seed to try and "recover" in next step
    end
end