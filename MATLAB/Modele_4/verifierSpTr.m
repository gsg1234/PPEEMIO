function [isInWorkspace] = verifierSpTr(x,y)
    parametres;
    funct1=sqrt((x+L)^2+y^2);
    funct2=sqrt((x-L)^2+y^2);
    if funct1<r_eq_min || funct1>r_eq_max
        isInWorkspace = 0;
        warning("Some points are not on the workspace")
    elseif funct2<r_eq_min || funct2>r_eq_max
        isInWorkspace = 0;
        warning("Some points are not on the workspace")
    else
        isInWorkspace = 1;
    end
end

