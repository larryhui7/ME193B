function dx = two_link_dynamics(~, x)

    beta = 0.01;   
    gamma = 0.01; 
    gl = 1;

    theta = x(1);  phi = x(2);  thdot = x(3);  phidot = x(4);

    D = [1 + 2*beta*(1-cos(phi)), -beta*(1-cos(phi));
         beta*(1-cos(phi)), -beta];

    Cqd = [-beta*sin(phi)*(phidot^2 - 2*thdot*phidot);
            beta*(thdot^2)*sin(phi)];

    Gq = [beta*gl*(sin(theta - phi - gamma) - sin(theta - gamma)) - gl*sin(theta - gamma);
          beta*gl*sin(theta - phi - gamma)];

    qdd = - D\(Cqd + Gq);

    dx = [thdot; phidot; qdd(1); qdd(2)];
end
