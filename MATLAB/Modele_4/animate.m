function [] = animate(Q1_total,Q3_total, trajectoire)
    figure('Name', 'Robot EMIO', 'Color', 'w');
    hold on; grid on; axis equal;
    xlim([-250 250]); ylim([-350 50]);
    parametres;
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

    for k = 1:1:size(Q1_total, 1)
        % --- CALCUL ROBOT 1 (Droit) ---
        t1 = Q1_total(k,1); d1_val = Q1_total(k,2);
        p0_1 = [L; 0];
        p1_1 = p0_1 + [r*cos(t1); r*sin(t1)];
        % L'actuateur est perpendiculaire au bras r (à cause du trotx(pi/2))
        p2_1 = p1_1 + [d1_val*cos(t1-pi/2); d1_val*sin(t1-pi/2)];
        p2_1aux=p2_1 - [100*cos(t1-pi/2); 100*sin(t1-pi/2)];
        p3_1 = p2_1 + [l*cos(t1-pi/2-pi/4); l*sin(t1-pi/2-pi/4)];
        
        % --- CALCUL ROBOT 3 (Gauche) ---
        t3 = Q3_total(k,1); d3_val = Q3_total(k,2);
        p0_3 = [-L; 0];
        p1_3 = p0_3 + [-r*cos(t3); r*sin(t3)]; % Adaptation th3 pour symétrie
        p2_3 = p1_3 + [d3_val*cos(t3+pi/2); -d3_val*sin(t3+pi/2)];
        p2_3aux=p2_3 - [100*cos(t3+pi/2); -100*sin(t3+pi/2)];
        p3_3 = p2_3 + [l*cos(t3+pi/2-pi/4); -l*sin(t3+pi/2-pi/4)];

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
        
        if trajectoire==1
        % Trajectoire (moyenne des deux points pour la précision)
            px = [px, (p3_1(1)+p3_3(1))/2];
            py = [py, (p3_1(2)+p3_3(2))/2];
            set(h_path, 'XData', px, 'YData', py);
        pause(0.1)
        end

        drawnow limitrate;
    end
end

