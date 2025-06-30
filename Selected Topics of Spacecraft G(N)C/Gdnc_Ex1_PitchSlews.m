clear
close all
%%
muEarth = 3.986e14;	% [m^3/s^2]	Earth's gravitational parameter
earthRadius = 6.378e6;	% [m] Earth's radius

orbitAlt = 500e3;	% [m] Orbit altitude (circular orbit)

stereoAngles = [10, 30]*pi/180;	% [rad]

%%
% Compute SC velocity:
satVel = sqrt(muEarth/(earthRadius+orbitAlt));	% [m/s]

% Distance on orbit patch from stereo angles to Nadir:
deltaPosNadir = orbitAlt .* tan(stereoAngles);	% [m]

% Distance on orbit patch for first slew (Nadir -> angle 1):
deltaPos1 = deltaPosNadir(1);

% Distance on orbit patch for second slew (angle 1 -> angle 2):
deltaPos2 = deltaPosNadir(2) - deltaPosNadir(1);

% Time between the stereo shots:
slewTime1 = deltaPos1 ./ satVel;	% [s]
slewTime2 = deltaPos2 ./ satVel;	% [s]

%% Plot 1: Schematic representation of viewing geometry (Nadir + Stereo angles)
figure('Name','Stereo Imaging Geometry');
hold on; axis equal; grid on;

% Plot Earth
theta = linspace(0, 2*pi, 500);
plot(earthRadius * cos(theta), earthRadius * sin(theta), 'b', 'DisplayName','Earth');

% Satellite position in orbit (single point)
satPos = [earthRadius + orbitAlt, 0];
plot(satPos(1), satPos(2), 'ko', 'MarkerFaceColor','k', 'DisplayName','Satellite');

% Viewing directions (vectors)
quiver(satPos(1), satPos(2), ...
    -earthRadius * tan(0), -earthRadius, 0, 'r', 'LineWidth', 1.5, 'DisplayName','Nadir');

quiver(satPos(1), satPos(2), ...
    -deltaPosNadir(1), -orbitAlt, 0, 'g', 'LineWidth', 1.5, 'DisplayName','Angle 1');

quiver(satPos(1), satPos(2), ...
    -deltaPosNadir(2), -orbitAlt, 0, 'm', 'LineWidth', 1.5, 'DisplayName','Angle 2');

legend('Location','southoutside');
xlabel('X [m]');
ylabel('Y [m]');
title('Satellite Imaging Geometry');

%% Plot 2: Temporal sequence of stereo acquisition
figure('Name','Slew Timing');
bar([1 2], [slewTime1 slewTime2]*1000, 'FaceColor', [0.2 0.6 0.8]);
set(gca,'xticklabel',{'Nadir → Angle 1','Angle 1 → Angle 2'});
ylabel('Slew Time [ms]');
title('Slew Times between Stereo Shots');
grid on;

%% 3D Visualization of Orbit and Stereo Imaging Directions
figure('Name','3D Orbit and Viewing Geometry');
hold on; grid on; axis equal;
xlabel('X [km]'); ylabel('Y [km]'); zlabel('Z [km]');
title('Satellite Orbit and Stereo Viewing Directions');

% Earth surface (sphere)
[xe, ye, ze] = sphere(50);
surf(earthRadius/1e3 * xe, earthRadius/1e3 * ye, earthRadius/1e3 * ze, ...
    'FaceColor','cyan','EdgeColor','none','FaceAlpha',0.3);

% Orbit path (circular in equatorial plane)
theta_orbit = linspace(0, 2*pi, 200);
orbitRadius = earthRadius + orbitAlt;
x_orbit = orbitRadius * cos(theta_orbit);
y_orbit = orbitRadius * sin(theta_orbit);
z_orbit = zeros(size(theta_orbit));
plot3(x_orbit/1e3, y_orbit/1e3, z_orbit/1e3, 'k--', 'DisplayName','Orbit');

% Satellite position at 0° true anomaly
satPos = [orbitRadius, 0, 0];

% Viewing directions
viewLength = 1.2 * orbitAlt;  % [m], length of view vectors
viewDirs = [-deltaPosNadir(1), -orbitAlt, 0;  % Angle 1
            -deltaPosNadir(2), -orbitAlt, 0;  % Angle 2
            0, -orbitAlt, 0];                % Nadir

colors = ['g', 'm', 'r'];
labels = {'Angle 1','Angle 2','Nadir'};

for i = 1:3
    viewVec = viewDirs(i,:) / norm(viewDirs(i,:)) * viewLength;
    quiver3(satPos(1)/1e3, satPos(2)/1e3, satPos(3)/1e3, ...
            viewVec(1)/1e3, viewVec(2)/1e3, viewVec(3)/1e3, ...
            'LineWidth',1.5, 'Color', colors(i), 'DisplayName', labels{i});
end

% Satellite marker
plot3(satPos(1)/1e3, satPos(2)/1e3, satPos(3)/1e3, 'ko', 'MarkerFaceColor','k', 'DisplayName','Satellite');

legend('Location','southoutside');
view(45, 25);  % Set good 3D viewing angle
