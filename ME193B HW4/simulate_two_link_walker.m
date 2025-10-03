function [t_sol, x_sol, t_I] = simulate_two_link_walker(N)
    
    x0 = [0.2065; 0.4130; -0.2052; -0.0172];
    
    options = odeset('Events', @(t, x) two_link_event(t, x));
    x_sol= []; t_sol = []; t_I = [];
    t0 = 0;
    
    for i = 1:N
        [t, x] = ode45(@(t, x) two_link_dynamics(t, x), [t0, t0+10], x0, options);
        x_sol = [x_sol; x];
        t_sol = [t_sol; t];
        t_I(i) = find(t_sol == t_sol(end), 1);
        t0 = t_sol(end);
        x0 = two_link_impactdynamics(x(end, :));
    end
end