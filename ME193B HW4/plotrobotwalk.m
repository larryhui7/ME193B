[t_sol, x_sol, t_I] = simulate_two_link_walker(10);
th = x_sol(:,1); phi = x_sol(:,2);

figure;
plot(t_sol, th); 
hold on;
plot(t_sol, phi);
xlabel("Time (s)"); ylabel("Angle (rad)");
title("Simulation of Walking Robot for N =10");
legend("theta", "phi");

animate_two_link_walker(t_sol, x_sol, 0.01, t_I)