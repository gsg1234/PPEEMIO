function animation_robot_ENIB_final()
    clc; close all;

    %% 1. PARAMÈTRES PHYSIQUES %%
    a0 = 10; a1 = 5; a2 = sqrt(2)*(10+5);
    
    % Matrices de transformation numériques
    transl_m = @(px,py) [1 0 0 px; 0 1 0 py; 0 0 1 0; 0 0 0 1];
    trotz_m = @(t) [cos(t) -sin(t) 0 0; sin(t) cos(t) 0 0; 0 0 1 0; 0 0 0 1];

    %% 2. DÉFINITION SYMBOLIQUE (FIX : Utilisation de sym) %%
    % On utilise sym() pour éviter l'erreur de "static workspace" dans les fonctions imbriquées
    x = sym('x', 'real'); y = sym('y', 'real'); th = sym('th', 'real');
    b1A = sym('b1A', 'real'); b1B = sym('b1B', 'real');
    tA = sym('tA', 'real'); tB = sym('tB', 'real');

    % Définition des chaînes cinématiques
    T1A = transl_m(a0,0);
    T2A = trotz_m(tA)*transl_m(a1,0);
    T3A = trotz_m(b1A)*transl_m(a2,0);
    T4A = trotz_m(-pi/2-tA-b1A);

    T1B = trotz_m(pi)*transl_m(a0,0);
    T2B = trotz_m(tB)*transl_m(a1,0);
    T3B = trotz_m(b1B)*transl_m(a2,0);
    T4B = trotz_m(pi/2-tB-b1B);

    Tsol = [cos(th), -sin(th), 0, x; sin(th), cos(th), 0, y; 0, 0, 1, 0; 0, 0, 0, 1];
    
    % Calcul des résidus (équations à annuler)
    eqs = [T1A*T2A*T3A*T4A - Tsol; T1B*T2B*T3B*T4B - Tsol];
    residuos = [eqs(1,4); eqs(2,4); eqs(5,4); eqs(6,4); eqs(1,1)];
    
    % Génération du solveur numérique optimisé
    f_solver = matlabFunction(residuos, 'Vars', {[x, y, th, b1A, b1B], tA, tB});

    %% 3. INTERFACE GRAPHIQUE (UI) %%
    fig = uifigure('Name', 'Contrôle Robot EMIO - ENIB', 'Position', [100 100 900 650]);
    
    % IMPORTANT : Utilisation de uiaxes pour la compatibilité uifigure
    ax = uiaxes(fig, 'Position', [100 250 700 350]);
    grid(ax, 'on'); 
    axis(ax, 'equal'); 
    hold(ax, 'on');
    
    % Ajustement des limites pour une visualisation optimale
    ax.XLim = [-50 50]; 
    ax.YLim = [-50 20];
    title(ax, 'Cinématique en Temps Réel');

    % Initialisation des objets graphiques (lignes et articulations)
    hA = plot(ax, [0, 0, 0, 0], [0, 0, 0, 0], '-ob', 'LineWidth', 3, 'MarkerFaceColor', 'b', 'DisplayName', 'Bras A');
    hB = plot(ax, [0, 0, 0, 0], [0, 0, 0, 0], '-or', 'LineWidth', 3, 'MarkerFaceColor', 'r', 'DisplayName', 'Bras B');
    hPlat = plot(ax, [0, 0], [0, 0], 'k--', 'LineWidth', 2, 'DisplayName', 'Plateforme');
    
    % Étiquette pour l'affichage des coordonnées
    lblPos = uilabel(fig, 'Position', [300 210 400 30], 'Text', 'Initialisation...', 'FontSize', 12, 'FontWeight', 'bold');

    %% 4. ÉTAT ET MISE À JOUR %%
    % Vecteur initial (seed) pour le solveur : [x, y, th, b1A, b1B]
    current_sol = [0, -15, -pi/2, -pi/4, pi/4]; 

    function update()
        t1A_val = sldA.Value;
        t1B_val = sldB.Value;
        
        % Résolution numérique du système
        options = optimoptions('fsolve', 'Display', 'off', 'TolFun', 1e-6);
        [new_sol, ~, exitflag] = fsolve(@(v) f_solver(v, t1A_val, t1B_val), current_sol, options);
        
        if exitflag > 0
            current_sol = new_sol; % Mise à jour de la solution pour la prochaine itération
            
            % Points Bras A : [Origine (0,0) -> Base(a0,0) -> Articulation -> Effecteur]
            xA = [0, a0, a0 + a1*cos(t1A_val), new_sol(1)];
            yA = [0, 0,  a1*sin(t1A_val),       new_sol(2)];
            
            % Points Bras B : [Origine (0,0) -> Base(-a0,0) -> Articulation -> Effecteur]
            xB = [0, -a0, -a0 + a1*cos(pi + t1B_val), new_sol(1)];
            yB = [0, 0,   a1*sin(pi + t1B_val),       new_sol(2)];
            
            % Mise à jour du graphique avec les nouvelles données
            hA.XData = xA; hA.YData = yA;
            hB.XData = xB; hB.YData = yB;
            hPlat.XData = [xA(4) xB(4)]; hPlat.YData = [yA(4) yB(4)];
            
            % Mise à jour de l'affichage numérique
            lblPos.Text = sprintf('X: %.2f | Y: %.2f | Th: %.2f° | b1A: %.2f° | b1B: %.2f°', ...
                new_sol(1), new_sol(2), rad2deg(new_sol(3)), rad2deg(new_sol(4)),rad2deg(new_sol(5)));
            
            drawnow limitrate;
        end
    end

    %% 5. CONTRÔLES (SLIDERS) %%
    sldA = uislider(fig, 'Position', [200 130 500 3], 'Limits', [-pi/2, 0], 'Value', 0);
    uilabel(fig, 'Position', [100 120 100 20], 'Text', 'Tita 1A (rad)');

    sldB = uislider(fig, 'Position', [200 70 500 3], 'Limits', [0, pi/2], 'Value', 0);
    uilabel(fig, 'Position', [100 60 100 20], 'Text', 'Tita 1B (rad)');

    % Définition des rappels (callbacks)
    sldA.ValueChangedFcn = @(s,e) update();
    sldB.ValueChangedFcn = @(s,e) update();

    % Premier tracé à l'ouverture
    update();
end