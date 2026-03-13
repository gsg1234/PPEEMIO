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
        verifierSpTr(new_sol(1),new_sol(2));
    else
        warning('Convergence failed at th1=%f. Keeping previous seed.', th1);
        q1 = [th1, 0]; 
        q3 = [th3, 0];
        new_seed = seed; % Keep old seed to try and "recover" in next step
    end
end

