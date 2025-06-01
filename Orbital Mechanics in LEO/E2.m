% Excercise 1

% constants
r0 = 6678137; % [m]
mu = 3.986 * 10^14; % [m^3/s^2]

% vectors
v_r0 = [r0; 0; 0];
v_v0 = [0; sqrt(mu/r0); 0];

a  = R_E + 300 * 10^3;
e  = 0.0001;
i  = deg2rad(98);
O  = deg2rad(10);
w  = deg2rad(10);
theta = deg2rad(10);

x0kep = [a; e; i; O; w; theta];
[x0] = kep2rv(x0kep);

% timespan
t0 = 0;
tf = 1 * 2*pi*sqrt(a^3/mu);

tspan = [t0, tf];

% State vector at t = 0
%x0 = [v_r0; v_v0];
%disp(x0)

%x0kep = rv2kep(x0);
%disp(x0kep)
%x0 = kep2rv(x0kep);

disp(x0)

[t,x] = ode45(@(t,x) odefcn(t,x,mu), tspan, x0);

function dxdt = odefcn(~,x,mu)
    r = x(1:3);
    v = x(4:6);
    r_norm = norm(r);
    a = -mu * r / r_norm^3;
    dxdt = [v; a];
end

n = length(t);
kep_hist = zeros(n, 6);

for k = 1:n
    kep_hist(k, :) = rv2kep(x(k, :)')';
end


labels = {'a [m]', 'e', 'i [rad]', '\Omega [rad]', '\omega [rad]', '\theta [rad]'};
figure;
for i = 1:6
    subplot(3,2,i);
    plot(t/3600, kep_hist(:,i));
    xlabel('Time [h]');
    ylabel(labels{i});
    grid on;
end
sgtitle('Zeitentwicklung der Kepler-Elemente');