% EMIO
clc, clear, close all;
a0=50;
a1=50;
a4=20;

dh = [
    %theta d         a     alfa       sigma
    1.000  0.000     a1    0.000       0;
    1.000  0.000     0     pi/2        0;
    0.000  1.000    0     -pi/2       1;
    1.000  0.000     a4    0.000       0];

R = SerialLink(dh,'name','EMIO');
q = [0,0,0,0,0,0];

% Límites de cada articulación
R.qlim(1,1:2) = [-185,  185]*pi/180;
R.qlim(2,1:2) = [-175,  60]*pi/180;
R.qlim(3,1:2) = [30,120];
R.qlim(4,1:2) = [-180,  180]*pi/180;

% offset
% Es un desplazamiento angular o lineal inicial aplicado a cada articulación.
% Útil cuando la referencia física de la articulación no coincide con el cero matemático del modelo.
R.offset = [0 0 0 0 0 0];

% base
% Es una matriz homogénea que representa la transformación del sistema de coordenadas global 
% al sistema de coordenadas de la base del robot.
% Por ahora asumimos uno genérico
R.base = transl(a0,0,0);

% tool
% Es una matriz homogénea, pero describe la transformación desde la última articulación
% hasta el effector. Asumimos uno genérico
R.tool = transl(-30, -30, 0)*trotz(-pi/2);

% [-limX, +limX, -limY, +limY, -limZ, +limZ]
workspace = [-3, 3, -3, 3, -0.2, 3];