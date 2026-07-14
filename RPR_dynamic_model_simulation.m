% Define parameters of the manipulator
l1 = 2;
l2 = 2;
l3 = 2;
d1 = 1;
d2 = 1;
d3 = 1;
m1 = 1;
m2 = 1;
m3 = 1;

% Define inertia parameters
Ic1zz = 0.1;
Ic2zz = 0.2;
Ic3zz = 0.3;
Ic1yy = 0.3;
Ic2yy = 0.7;
Ic3yy = 0.2;
Ic2xy = 0.9;
Ic3xy = 0.4;
Ic2xx = 0.6;
Ic3xx = 0.8;
Ic3yz = 0.1;
Ic3xz = 0.5;

% Define gravity acceleration
g0 = 9.81;

% Define time parameters
deltat = 0.001;
t = 0:deltat:5;

% Define sinusoidal motions for joint positions, velocities, and accelerations
Q1 = (pi/6)*sin((pi/2)*t);
Q2 = (pi/6)*sin((pi/2)*t);
Q3 = (pi/6)*sin((pi/2)*t);
dQ1 = diff(Q1)/deltat;
dQ1 = [dQ1,dQ1(end)];
dQ2 = diff(Q2)/deltat;
dQ2 = [dQ2,dQ2(end)];
dQ3 = diff(Q3)/deltat;
dQ3 = [dQ3,dQ3(end)];
ddQ1 = diff(dQ1)/deltat;
ddQ1 =[ddQ1,ddQ1(end)];
ddQ2 = diff(dQ2)/deltat;
ddQ2 =[ddQ2,ddQ2(end)];
ddQ3 = diff(dQ3)/deltat;
ddQ3 =[ddQ3,ddQ3(end)];

% Initialize matrix to store generalized forces over time
TAU = zeros(3, length(t));

% Loop through each time step
for i=1:length(t)
    % Set joint positions, velocities, and accelerations
    q1 = Q1(i);
    q2 = Q2(i);
    q3 = Q3(i);
    dq1 = dQ1(i);
    dq2 = dQ2(i);
    dq3 = dQ3(i);
    ddq1 = ddQ1(i);
    ddq2 = ddQ2(i);
    ddq3 = ddQ3(i);
    
    % Calculate generalized forces using inverse dynamic model
    TAU(:, i) = eval(tau);
end

% Plot the generalized forces over time for each joint
figure;
plot(t,TAU(1,:))
xlabel('time [s]');
ylabel('\tau(t)');
hold on
plot (t, TAU (2,:))
hold on
plot (t, TAU (3,:))
legend ('\tau_l(t)','\tau_2(t)','\tau_3(t)')
grid
