clc; clear; close all;

% Désactiver l'avertissement du système non carré (pour fsolve)
warning('off', 'optim:rootfind:SystemNotSquare');

%% Paramètres Géométriques
L = 100;
r = 50;
d1 = 200;
d3 = 200;
lt=10;
pasos = 100;
% AUTRES PARAMÈTRES
anim=2;

% Vecteurs de mouvement (Trajectoires)
tA_tray = linspace(-1.4302, -0.6970, pasos);
tB_tray = linspace(-1.4302, -0.6970, pasos);


% Initialisation des robots (Scripts externes)
robot1;
robot3;

th1 = tA_tray(1);
th3 = tB_tray(1);

%% RÉSOLUTION SYMBOLIQUE %%
x = sym('x', 'real'); y = sym('y', 'real'); th1e = sym('th1e', 'real'); th3e = sym('th3e', 'real');
d1 = sym('d1', 'real'); d3=sym('d3', 'real');
th1s = sym('th1s', 'real'); th3s = sym('th3s', 'real');

T1B = transl(L,0,0);
T11 = trotz(th1s)*transl(r,0,0)*trotx(pi/2);
T12 = transl(0,0,d1);
T1T = trotx(-pi/2)*trotz(-3*pi/4)*transl(lt,0,0)*trotz(-pi/2);

T3B = troty(pi)*transl(L,0,0);
T31 = trotz(th3s)*transl(r,0,0)*trotx(pi/2);
T32 = transl(0,0,d3);
T3T = trotx(-pi/2)*trotz(-3*pi/4)*transl(lt,0,0)*trotz(-pi/2)*trotx(pi);

Tsol1 = [cos(th1e), -sin(th1e), 0, x; sin(th1e), cos(th1e), 0, y; 0, 0, 1, 0; 0, 0, 0, 1];
Tsol3 = [cos(th3e), -sin(th3e), 0, x; sin(th3e), cos(th3e), 0, y; 0, 0, 1, 0; 0, 0, 0, 1];
eqs = [T1B*T11*T12*T1T - Tsol1; T3B*T31*T32*T3T - Tsol3];

% Résidus pour fsolve (système sur-contraint pour assurer la coïncidence)
residuos = [eqs(1,4); eqs(2,4); eqs(5,4); eqs(6,4)];
f_solver = matlabFunction(residuos, 'Vars', {[x, y, d1, d3], th1s, th3s});
f_solverinv=matlabFunction(residuos, 'Vars',{[th1s, th3s, d1, d3], x, y});

% Valeurs initiales (Seed)
seed = [0, -150, 100, 100];

%% CALCUL DES TRAJECTOIRES GD %%
q1A1=zeros(2*pasos,2); q1A2=zeros(2*pasos,2); q1A3=zeros(2*pasos,2); 
q3A1=zeros(2*pasos,2); q3A2=zeros(2*pasos,2); q3A3=zeros(2*pasos,2);

% ÉTAPE 1 : Mouvement de th1 uniquement
disp('Calcul Étape 1 : th1 en mouvement...');
for i = 1:pasos
    [q1A1(i,:), q3A1(i,:),seed] = calculer_GD(tA_tray(i), th3, f_solver, seed);
    [q1A1(i+pasos,:), q3A1(i+pasos,:),seed] = calculer_GD(tA_tray(pasos+1-i), th3, f_solver, seed);
end

% ÉTAPE 2 : Mouvement de th3 uniquement
disp('Calcul Étape 2 : th3 en mouvement...');
for i = 1:pasos
    [q1A2(i,:), q3A2(i,:),seed] = calculer_GD(th1, tB_tray(i), f_solver, seed);
    [q1A2(i+pasos,:), q3A2(i+pasos,:),seed] = calculer_GD(th1, tB_tray(pasos+1-i), f_solver, seed);
end

% ÉTAPE 3 : Mouvement simultané
disp('Calcul Étape 3 : th1 et th3 en mouvement...');
for i = 1:pasos
    [q1A3(i,:), q3A3(i,:),seed] = calculer_GD(tA_tray(i), tB_tray(i), f_solver, seed);
    [q1A3(i+pasos,:), q3A3(i+pasos,:),seed] = calculer_GD(tA_tray(pasos+1-i), tB_tray(pasos+1-i), f_solver, seed);
end

% Fusion des trajectoires
Q1_total = [q1A1; q1A2; q1A3];
Q3_total = [q3A1; q3A2; q3A3];
anim=2;
%% ANIMATION %%
if anim==1
    figure('Name', 'Animation Robots ENIB');
    R1.plot(Q1_total(1,:), 'delay', 0, 'nojaxes', 'noshadow', 'workspace', [-200 200 -300 100 -10 100]);
    hold on;
    R3.plot(Q3_total(1,:), 'delay', 0, 'nojaxes', 'noshadow', 'workspace', [-200 200 -300 100 -10 100]);
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
%% ANIMATION WITH MATLAB PLOTS %%
elseif anim==2
    figure('Name', 'Robot EMIO - Rendu Natif', 'Color', 'w');
    hold on; grid on; axis equal;
    xlim([-250 250]); ylim([-350 50]);
    
    % Tracé des bases et des cercles de débattement (statique)
    theta_c = linspace(0, 2*pi, 100);
    plot(L + r*cos(theta_c), r*sin(theta_c), 'k:', 'LineWidth', 0.5); % Cercle A
    plot(-L + r*cos(theta_c), r*sin(theta_c), 'k:', 'LineWidth', 0.5); % Cercle B
    plot([-L, L], [0, 0], 'k--', 'LineWidth', 1); % Ligne de base
    
    % Initialisation des segments du Robot 1 (Droit - Bleu)
    h_r1_prox = plot([0 0], [0 0], 'b', 'LineWidth', 4); % Bras r
    h_r1_cyl  = plot([0 0], [0 0], 'Color', [0.2 0.2 0.8], 'LineWidth', 8); % Corps vérin
    h_r1_rod  = plot([0 0], [0 0], 'Color', [0.6 0.6 0.6], 'LineWidth', 3); % Tige vérin
    h_r1_eff = plot([0 0], [0 0], 'Color', [0.6 0.6 0.6], 'LineWidth', 3); %eff
    h_r1_effadd = plot([0 0], [0 0], 'Color', [1.0 1.0 1.0], 'LineWidth', 3); %effh_r1_effadd = plot([0 0], [0 0], 'Color', [1.0 1.0 1.0], 'LineWidth', 3); %eff
    
    % Initialisation des segments du Robot 3 (Gauche - Rouge)
    h_r3_prox = plot([0 0], [0 0], 'r', 'LineWidth', 4);
    h_r3_cyl  = plot([0 0], [0 0], 'Color', [0.8 0.2 0.2], 'LineWidth', 8);
    h_r3_rod  = plot([0 0], [0 0], 'Color', [0.6 0.6 0.6], 'LineWidth', 3);
    h_r3_eff = plot([0 0], [0 0], 'Color', [0.6 0.6 0.6], 'LineWidth', 3);
    h_r3_effadd = plot([0 0], [0 0], 'Color', [1.0 1.0 1.0], 'LineWidth', 3); %eff

    % Trajectoire de l'outil
    h_path = plot(NaN, NaN, 'g', 'LineWidth', 2);
    px = []; py = [];

    for k = 1:2:size(Q1_total, 1)
        % --- CALCUL ROBOT 1 (Droit) ---
        t1 = Q1_total(k,1); d1_val = Q1_total(k,2);
        p0_1 = [L; 0];
        p1_1 = p0_1 + [r*cos(t1); r*sin(t1)];
        % L'actuateur est perpendiculaire au bras r (à cause du trotx(pi/2))
        p2_1 = p1_1 + [d1_val*cos(t1-pi/2); d1_val*sin(t1-pi/2)];
        p2_1aux=p2_1 - [100*cos(t1-pi/2); 100*sin(t1-pi/2)];
        p3_1 = p2_1 + [lt*cos(t1-pi/2-pi/4); lt*sin(t1-pi/2-pi/4)];
        
        % --- CALCUL ROBOT 3 (Gauche) ---
        t3 = Q3_total(k,1); d3_val = Q3_total(k,2);
        p0_3 = [-L; 0];
        p1_3 = p0_3 + [-r*cos(t3); r*sin(t3)]; % Adaptation th3 pour symétrie
        p2_3 = p1_3 + [d3_val*cos(t3+pi/2); -d3_val*sin(t3+pi/2)];
        p2_3aux=p2_3 - [100*cos(t3+pi/2); -100*sin(t3+pi/2)];
        p3_3 = p2_3 + [lt*cos(t3+pi/2-pi/4); -lt*sin(t3+pi/2-pi/4)];

        % --- MISE À JOUR GRAPHIQUE ---
        % Robot 1
        set(h_r1_prox, 'XData', [p0_1(1) p1_1(1)], 'YData', [p0_1(2) p1_1(2)]);
        % Effet télescopique : le corps fait 50% de l'extension max
        p_mid1 = p1_1 + [100*cos(t1-pi/2); 100*sin(t1-pi/2)];
        set(h_r1_cyl, 'XData', [p1_1(1) p_mid1(1)], 'YData', [p1_1(2) p_mid1(2)]);
        set(h_r1_rod, 'XData', [p1_1(1) p2_1(1)], 'YData', [p1_1(2) p2_1(2)]);
        set(h_r1_eff,'XData', [p2_1(1) p3_1(1)], 'YData', [p2_1(2) p3_1(2)]);
        set(h_r1_effadd,'XData', [p1_1(1) p2_1aux(1)], 'YData', [p1_1(2) p2_1aux(2)]);

        % Robot 3
        set(h_r3_prox, 'XData', [p0_3(1) p1_3(1)], 'YData', [p0_3(2) p1_3(2)]);
        p_mid3 = p1_3 + [100*cos(t3+pi/2); -100*sin(t3+pi/2)];
        set(h_r3_cyl, 'XData', [p1_3(1) p_mid3(1)], 'YData', [p1_3(2) p_mid3(2)]);
        set(h_r3_rod, 'XData', [p1_3(1) p2_3(1)], 'YData', [p1_3(2) p2_3(2)]);
        set(h_r3_eff,'XData', [p2_3(1) p3_3(1)], 'YData', [p2_3(2) p3_3(2)]);
        set(h_r3_effadd,'XData', [p1_3(1) p2_3aux(1)], 'YData', [p1_3(2) p2_3aux(2)]);
        
        % Trajectoire (moyenne des deux points pour la précision)
        % px = [px, (p3_1(1)+p3_3(1))/2];
        % py = [py, (p3_1(2)+p3_3(2))/2];
        % set(h_path, 'XData', px, 'YData', py);
        pause(0.1)

        drawnow limitrate;
    end
else
    disp('Calculs finis');
end


%% FONCTION AUXILIAIRE %%
function [q1, q3, new_seed] = calculer_GD(th1, th3, f_solver, seed)
    % lsqnonlin minimizes the vector returned by f_solver
    
    lb = [-500, -500, 100, 100];   % d1 cannot be negative
    ub = [ 500,  0,  200, 200]; % d1 max length 150
    
    
    options = optimoptions('lsqnonlin', 'Display', 'off');
    options.Algorithm = 'trust-region-reflective';
    [new_sol, resnorm, ~, exitflag] = lsqnonlin(@(v) f_solver(v, th1, th3), seed, lb, ub, options);
    %resnorm
    if exitflag > 0
        q1 = [th1, new_sol(3)];
        q3 = [th3, new_sol(4)];
        new_seed = new_sol; % Pass the solution back to be the next seed
    else
        warning('Convergence failed at th1=%f. Keeping previous seed.', th1);
        q1 = [th1, 0, 0]; 
        q3 = [th3, 0, 0];
        new_seed = seed; % Keep old seed to try and "recover" in next step
    end
end

function [q1, q3, new_seed] = calculer_GI(x, y, f_solver, seed)
    % lsqnonlin minimizes the vector returned by f_solver
    
    lb = [-1.4302, -1.4302, 100, 100];   % d1 cannot be negative
    ub = [-0.6970,  -0.6970,  200, 200]; % d1 max length 150
    
    
    options = optimoptions('lsqnonlin', 'Display', 'off');
    options.Algorithm = 'trust-region-reflective';
    [new_sol, resnorm, ~, exitflag] = lsqnonlin(@(v) f_solver(v, x, y), seed, lb, ub, options);
    %resnorm
    if exitflag > 0
        q1 = [th1, new_sol(3)];
        q3 = [th3, new_sol(4)];
        new_seed = new_sol; % Pass the solution back to be the next seed
    else
        warning('Convergence failed at th1=%f. Keeping previous seed.', th1);
        q1 = [th1, 0, 0]; 
        q3 = [th3, 0, 0];
        new_seed = seed; % Keep old seed to try and "recover" in next step
    end
end
