function T=DH(a, alpha, d, theta)

    threshold = 1e-10;

    T=[cos(theta)   -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta);...
       sin(theta)   cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)  a*sin(theta);...
       0            sin(alpha)              cos(alpha)              d;...
       0            0                       0                       1];

    % this part has been added to adjust numerical errors 
    for i=1:size(T,1)
        for j=1:size(T,2)
            coeff_ij = eval(coeffs(T(i,j)));
            if (abs(coeff_ij) < threshold)
                T(i,j) = 0;
            end
        end
    end
end