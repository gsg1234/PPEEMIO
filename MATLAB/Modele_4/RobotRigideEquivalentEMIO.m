clc; clear; close all;

parametres;

tA_tray = linspace(-0.6972,-1.0, pasos/4);
tB_tray = linspace(-0.6972,-1.0, pasos/4);

th1 = tA_tray(1);
th3 = tB_tray(1);


%% RÉSOLUTION SYMBOLIQUE %%
robot;

% Valeurs initiales (Seed)
seed = [0, -150, 100, 100];

%% CALCUL DES TRAJECTOIRES GD %%
q1A1=zeros(pasos/2,2); q1A2=zeros(pasos/2,2); q1A3=zeros(pasos/2,2); 
q3A1=zeros(pasos/2,2); q3A2=zeros(pasos/2,2); q3A3=zeros(pasos/2,2);

% ÉTAPE 1 : Mouvement de th1 uniquement
disp('Calcul Étape 1 : th1 en mouvement...');
for i = 1:pasos/4
    [q1A1(i,:), q3A1(i,:),seed] = calculer_GD(tA_tray(i), th3, f_solver, seed);
    [q1A1(i+pasos/4,:), q3A1(i+pasos/4,:),seed] = calculer_GD(tA_tray(pasos/4+1-i), th3, f_solver, seed);
end

% ÉTAPE 2 : Mouvement de th3 uniquement
disp('Calcul Étape 2 : th3 en mouvement...');
for i = 1:pasos/4
    [q1A2(i,:), q3A2(i,:),seed] = calculer_GD(th1, tB_tray(i), f_solver, seed);
    [q1A2(i+pasos/4,:), q3A2(i+pasos/4,:),seed] = calculer_GD(th1, tB_tray(pasos/4+1-i), f_solver, seed);
end

% ÉTAPE 3 : Mouvement simultané
disp('Calcul Étape 3 : th1 et th3 en mouvement...');
for i = 1:pasos/4
    [q1A3(i,:), q3A3(i,:),seed] = calculer_GD(tA_tray(i), tB_tray(i), f_solver, seed);
    [q1A3(i+pasos/4,:), q3A3(i+pasos/4,:),seed] = calculer_GD(tA_tray(pasos/4+1-i), tB_tray(pasos/4+1-i), f_solver, seed);
end

% Fusion des trajectoires
Q1_total = [q1A1; q1A2; q1A3];
Q3_total = [q3A1; q3A2; q3A3];

animate(Q1_total,Q3_total,1);


%% GI CIRCLE %%

t=linspace(0,2*pi,pasos);
x_tray=20*cos(t);
y_tray=20*sin(t)-120;

Q1_GI=zeros(pasos,2);
Q3_GI=zeros(pasos,2);
for i=1:pasos
    [Q1_GI(i,:),Q3_GI(i,:),seed]=calculer_GI(x_tray(i),y_tray(i),f_solverinv,seed);
end

animate(Q1_GI,Q3_GI,1);

%% GI CARRE %%

x_tray=linspace(-20,20,pasos/4);
y_tray=linspace(-100,-140,pasos/4);

Q1_GI=zeros(pasos,2);
Q3_GI=zeros(pasos,2);
for i=1:pasos/4
    [Q1_GI(i,:),Q3_GI(i,:),seed]=calculer_GI(x_tray(i),-100,f_solverinv,seed);
end
for i=1:pasos/4
    [Q1_GI(i+pasos/4,:),Q3_GI(i+pasos/4,:),seed]=calculer_GI(20,y_tray(i),f_solverinv,seed);
end
for i=1:pasos/4
    [Q1_GI(i+pasos/2,:),Q3_GI(i+pasos/2,:),seed]=calculer_GI(x_tray(pasos/4+1-i),-140,f_solverinv,seed);
end
for i=1:pasos/4
    [Q1_GI(i+3/4*pasos,:),Q3_GI(i+3/4*pasos,:),seed]=calculer_GI(-20,y_tray(pasos/4+1-i),f_solverinv,seed);
end

animate(Q1_GI,Q3_GI,1);

%% GI LIMIT %%

x1_tray=linspace(0,82.0711,pasos/4);
x2_tray=linspace(82.0711,0,pasos/4);
x3_tray=linspace(0,-82.0711,pasos/4);
x4_tray=linspace(-82.0711,0,pasos/4);

y1_tray=-sqrt(r_eq_min^2-(x1_tray-L).^2);
y2_tray=-sqrt(r_eq_max^2-(x2_tray+L).^2);
y3_tray=-sqrt(r_eq_max^2-(x3_tray-L).^2);
y4_tray=-sqrt(r_eq_min^2-(x4_tray+L).^2);

x_tray=[x1_tray,x2_tray,x3_tray,x4_tray];
y_tray=[y1_tray,y2_tray,y3_tray,y4_tray];

Q1_GI=zeros(pasos,2);
Q3_GI=zeros(pasos,2);
for i=1:pasos
    [Q1_GI(i,:),Q3_GI(i,:),seed]=calculer_GI(x_tray(i),y_tray(i),f_solverinv,seed);
end

animate(Q1_GI,Q3_GI,1);

