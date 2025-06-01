%% Orbital mechanics in LEO - Exercise 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Author: Constantin Traub                                                %
% Date: 30.05.2024                                                        %
% Institute of Space Systems                                              %
% University of Stuttgart                                                 %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
clc
close all

% Definition of all relevant constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mu_E = 3.986e14;                                                           % m^3/s^2 gravitational constant of Earth
R_E  = 6378137;                                                            % m Equatorial radius of the Earth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Definition of the initial conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a  = R_E + 300 * 10^3;
e  = 0.0001;
i  = deg2rad(98);
O  = deg2rad(10);
w  = deg2rad(10);
theta = deg2rad(10);

kep0 = [a; e; i; O; w; theta];
[x0] = kep2rv(kep0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Numerical integration of the state vector %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 1 * 2*pi*sqrt(a^3/mu_E);                                                  % s One orbital period
tspan = [0 tf];
opts = odeset('Maxstep', 10, 'RelTol',1e-4, 'AbsTol',1e-6);
[tout,xout] = ode45(@(tode,x) K2B(tode,x,mu_E),tspan, x0, opts);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plotting of trajectory %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
plot3(xout(:,1),xout(:,2),xout(:,3),'k')
grid on
xlabel('x ECI [m]');ylabel('y ECI [m]');zlabel('z ECI [m]')

% BONUS: Plotting of the Earth' surface
hold on
load('topo.mat', 'topo', 'topomap1');
[X,Y,Z] = sphere;
props.FaceColor= 'texture';
props.EdgeColor = 'none';
props.FaceLighting = 'phong';
props.CData = topo;
s = surface(R_E*X,R_E*Y,R_E*Z,props);

% BONUS 2: Plotting the equatorial plane
x = -R_E:R_E/4:R_E;
y = -R_E:R_E/4:R_E;
[X,Y] = meshgrid(x,y);
Z = zeros(length(x),length(y));
surf(X,Y,Z,'FaceColor','none');

% BONUS 3: Plot the ECI reference frame
vec_length = 1.25*R_E;
x_ECI = [vec_length,0,0];
Y_ECI = [0,vec_length,0];
Z_ECI = [0 0 vec_length];
orig = [0,0,0];
plot3([orig(1) x_ECI(1)],[orig(2) x_ECI(2)],[orig(3) x_ECI(3)],'k','LineWidth',1);
plot3([orig(1) Y_ECI(1)],[orig(2) Y_ECI(2)],[orig(3) Y_ECI(3)],'k','LineWidth',1);
plot3([orig(1) Z_ECI(1)],[orig(2) Z_ECI(2)],[orig(3) Z_ECI(3)],'k','LineWidth',1);
view(135,30)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% State vector to keplerian element conversion %%%%%%%%%%%%%%%%%%%%%%%%%%%%
kepout = zeros(length(xout(:,1)),6);
for i = 1:1:length(xout(:,1))
    kep = rv2kep(xout(i,:));
    kepout(i,:) = kep;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plotting of the keplerian elements %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
subplot(6,1,1)
plot(tout./3600,kepout(:,1)./1000)
grid on
ylabel('a [km]')
subplot(6,1,2)
plot(tout./3600,kepout(:,2))
grid on
ylabel('e [-]')
subplot(6,1,3)
plot(tout./3600,kepout(:,3))
grid on
ylabel('i [rad]')
subplot(6,1,4)
plot(tout./3600,kepout(:,4))
grid on
ylabel('\Omega [rad]')
subplot(6,1,5)
plot(tout./3600,kepout(:,5))
grid on
ylabel('\omega [rad]')
subplot(6,1,6)
plot(tout./3600,kepout(:,6))
grid on
ylabel('\theta [rad]')
xlabel('Time [h]')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dxdt = K2B(~,x,mu)

r = [x(1); x(2); x(3)];
v = [x(4); x(5); x(6)];
dxdt = [v; (-mu/norm(r)^3)*r];                                             % Keplerian dynamics

end
