%% Orbital mechanics in LEO - Exercise 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Author: Constantin Traub                                                %
% Date: 18.04.2024                                                        %
% Institute of Space Systems                                              %
% University of Stuttgart                                                 %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
clc
close all

% Definition of all relevant constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mu_E = 3.986e14;                                                           % m^3/s^2 gravitational constant of Earth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Definition of the initial conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r_norm0 = 6378.137e3 + 300e3;                                                % m
r0 = [r_norm0; 0; 0];
v0 = [0; sqrt(mu_E/r_norm0); 0];
x0 = [r0; v0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Numerical integration of the state vector %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 90 * 60;                                                              % s 
tspan = [0 tf];
opts = odeset('Maxstep', 10, 'RelTol',1e-4, 'AbsTol',1e-6);
[tout,xout] = ode45(@(tode,x) K2B(tode,x,mu_E),tspan, x0, opts);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plotting of the results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
s = surface(6378137*X,6378137*Y,6378137*Z,props);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function dxdt = K2B(~,x,mu)

r = [x(1); x(2); x(3)];
v = [x(4); x(5); x(6)];

dxdt = [v; (-mu/norm(r)^3)*r];                                             % Keplerian dynamics    
 
% % Also:
% dxdt = [x(4); x(5); x(6); (-mu/norm(x(1:3))^3)*x(1:3)];                    % Keplerian dynamics
       
end
