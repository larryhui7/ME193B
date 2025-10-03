function x_plus = two_link_impactdynamics(x_minus)
    th = x_minus(1); 
    A = [ -1,  0,  0, 0;
          -2,  0,  0, 0;
           0,  0,  cos(2*th), 0;
           0,  0,  cos(2*th)*(1 - cos(2*th)), 0 ];
    x_plus = A * x_minus.';
    x_plus = x_plus';
end

