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