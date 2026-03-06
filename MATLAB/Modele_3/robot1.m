% EMIO

dh = [
    %theta d         a     alfa       sigma
    0.000  0.000    r      0.000       0;
    0.000  0.000    0.000   pi/2        0;
    0.000  0.000    0.000   0.000        1];

R1 = SerialLink(dh,'name','EMIOB1');
q1 = [0,0,d1];

% Límites de cada articulación
R1.qlim(1,1:2) = [-185,  185]*pi/180;
R1.qlim(2,1:2) = [-175,  60]*pi/180;
R1.qlim(3,1:2) = [30,120];

% offset
% Es un desplazamiento angular o lineal inicial aplicado a cada articulación.
% Útil cuando la referencia física de la articulación no coincide con el cero matemático del modelo.
R1.offset = [0 0 0];

% base
% Es una matriz homogénea que representa la transformación del sistema de coordenadas global 
% al sistema de coordenadas de la base del robot.
% Por ahora asumimos uno genérico
R1.base = transl(L,0,0);

% tool
% Es una matriz homogénea, pero describe la transformación desde la última articulación
% hasta el effector. Asumimos uno genérico
%R1.tool = trotx(-pi/2)*trotz(-pi/4);
R1.tool = trotx(-pi/2)*trotz(-3*pi/4)*transl(10, 0, 0)*trotz(pi/2)*transl(5,0,0);

% [-limX, +limX, -limY, +limY, -limZ, +limZ]
workspace = [-3, 3, -3, 3, -0.2, 3];

R1.model3d = 'model3d';