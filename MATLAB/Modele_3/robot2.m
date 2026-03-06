% EMIO

dh = [
    %theta d         a     alfa       sigma
    0.000  0.000    r      0.000       0;
    0.000  0.000    0.000   pi/2        0;
    0.000  0.000    0.000   0.000        1];

R2 = SerialLink(dh,'name','EMIOB2');
q3 = [0,0,a3];

% Límites de cada articulación
R2.qlim(1,1:2) = [-185,  185]*pi/180;
R2.qlim(2,1:2) = [-175,  60]*pi/180;
R2.qlim(3,1:2) = [30,120];

% offset
% Es un desplazamiento angular o lineal inicial aplicado a cada articulación.
% Útil cuando la referencia física de la articulación no coincide con el cero matemático del modelo.
R2.offset = [0 0 0];

% base
% Es una matriz homogénea que representa la transformación del sistema de coordenadas global 
% al sistema de coordenadas de la base del robot.
% Por ahora asumimos uno genérico
R2.base = troty(pi/2)*transl(L,0,0);

% tool
% Es una matriz homogénea, pero describe la transformación desde la última articulación
% hasta el effector. Asumimos uno genérico
R2.tool = trotx(-pi/2)*trotz(-3*pi/4)*transl(10, 0, 0)*trotz(pi/2)*transl(5,0,0)*trotx(pi/2);

% [-limX, +limX, -limY, +limY, -limZ, +limZ]
workspace = [-3, 3, -3, 3, -0.2, 3];