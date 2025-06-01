% Excercise 1

% constants
r0 = 6678137; % [m]
mu = 3.986 * 10^14; % [m^3/s^2]

% vectors
v_r0 = [r0; 0; 0];
v_v0 = [0; sqrt(mu/r0); 0];


% timespan
t0 = 0;
tf = 90*60;

tspan = [t0, tf];

% State vector at t = 0
x0 = [v_r0; v_v0];

[t,x] = ode45(@(t,x) odefcn(t,x,mu), tspan, x0);

function dxdt = odefcn(~,x,mu)
dxdt(1,1) = x(4);
dxdt(2,1) = x(5);
dxdt(3,1) = x(6);
r = sqrt(x(1)^2+x(2)^2+x(3)^2);
dxdt(4,1) = - mu/r^3*x(1);
dxdt(5,1) = - mu/r^3*x(2);
dxdt(6,1) = - mu/r^3*x(3);
end


% figure
figure;
plot3(x(:,1), x(:,2), x(:,3), 'b')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Satellite trajectory in ECI frame')
grid on
axis equal