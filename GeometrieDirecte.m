clc; clear; close all;

% Désactiver l'avertissement du système non carré (pour fsolve)
warning('off', 'optim:rootfind:SystemNotSquare');

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
robot1;
robot3;

tA = -pi/2;
tB = -pi/2;

%% RÉSOLUTION SYMBOLIQUE %%
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

% Résidus pour fsolve (système sur-contraint pour assurer la coïncidence)
residuos = [eqs(1,4); eqs(2,4); eqs(5,4); eqs(6,4); eqs(1,1); eqs(1,2); eqs(5,1); eqs(5,2)];
f_solver = matlabFunction(residuos, 'Vars', {[x, y, th, b1A, b1B, L1], tAs, tBs});

% Valeurs initiales (Seed)
seed = [0, -15, -pi/2, -pi/4, -pi/4, 20];

%% CALCUL DES TRAJECTOIRES %%
q1A1=zeros(2*pasos,3); q1A2=zeros(2*pasos,3); q1A3=zeros(2*pasos,3); 
q3A1=zeros(2*pasos,3); q3A2=zeros(2*pasos,3); q3A3=zeros(2*pasos,3);

% ÉTAPE 1 : Mouvement de tA uniquement
disp('Calcul Étape 1 : tA en mouvement...');
for i = 1:pasos
    [q1A1(i,:), q3A1(i,:)] = calculer_pas(tA_tray(i), tB, f_solver, seed, a3);
    [q1A1(i+pasos,:), q3A1(i+pasos,:)] = calculer_pas(tA_tray(pasos+1-i), tB, f_solver, seed, a3);
end

% ÉTAPE 2 : Mouvement de tB uniquement
disp('Calcul Étape 2 : tB en mouvement...');
for i = 1:pasos
    [q1A2(i,:), q3A2(i,:)] = calculer_pas(tA, tB_tray(i), f_solver, seed, a3);
    [q1A2(i+pasos,:), q3A2(i+pasos,:)] = calculer_pas(tA, tB_tray(pasos+1-i), f_solver, seed, a3);
end

% ÉTAPE 3 : Mouvement simultané
disp('Calcul Étape 3 : tA et tB en mouvement...');
for i = 1:pasos
    [q1A3(i,:), q3A3(i,:)] = calculer_pas(tA_tray(i), tB_tray(i), f_solver, seed, a3);
    [q1A3(i+pasos,:), q3A3(i+pasos,:)] = calculer_pas(tA_tray(pasos+1-i), tB_tray(pasos+1-i), f_solver, seed, a3);
end

% Fusion des trajectoires
Q1_total = [q1A1; q1A2; q1A3];
Q3_total = [q3A1; q3A2; q3A3];

%% ANIMATION %%
figure('Name', 'Animation Robots ENIB');
R1.plot(Q1_total(1,:), 'delay', 0, 'nojaxes', 'noshadow', 'workspace', [-100 150 -150 100 -10 100]);
hold on;
R3.plot(Q3_total(1,:), 'delay', 0, 'nojaxes', 'noshadow', 'workspace', [-100 150 -150 100 -10 100]);
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
function [q1, q3] = calculer_pas(tA, tB, f_solver, seed, a3)
    % Options silencieuses pour le solveur
    options = optimoptions('fsolve', 'Algorithm', 'levenberg-marquardt', 'Display', 'off');
    [new_sol, ~, exitflag] = fsolve(@(v) f_solver(v, tA, tB), seed, options);
    
    if exitflag > 0
        % Correction du signe de L1 (translation) et normalisation
        if new_sol(6) < 0
            new_sol(6) = -new_sol(6);
            new_sol(4) = new_sol(4) + pi;
        end
        if new_sol(4) > (2*pi)
            new_sol(4) = new_sol(4) - 2*pi;
        end
        q1 = [tA, new_sol(4), new_sol(6)];
        q3 = [tB, new_sol(5), a3];
    else
        warning('Erreur de convergence du solveur à tA=%f, tB=%f', tA, tB);
        q1 = [tA, 0, 0]; q3 = [tB, 0, a3];
    end
end