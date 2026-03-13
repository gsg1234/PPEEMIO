function [q1, q3, new_seed] = calculer_GI(x, y, f_solver, seed)
    % lsqnonlin minimizes the vector returned by f_solver
    
    lb = [-pi/2, -pi/2, 100, 100];   % d1 cannot be negative
    ub = [0,  0,  200, 200]; % d1 max length 150
    
    
    options = optimoptions('lsqnonlin', 'Display', 'off');
    options.Algorithm = 'trust-region-reflective';
    [new_sol, resnorm, ~, exitflag] = lsqnonlin(@(v) f_solver(v, x, y), seed, lb, ub, options);
    %resnorm
    if exitflag > 0
        q1 = [new_sol(1), new_sol(3)];
        q3 = [new_sol(2), new_sol(4)];
        new_seed = new_sol; % Pass the solution back to be the next seed
        verifierSpTr(x,y);
    else
        warning('Convergence failed at th1=%f. Keeping previous seed.', th1);
        q1 = [0, 0]; 
        q3 = [0, 0];
        new_seed = seed; % Keep old seed to try and "recover" in next step
    end
end
