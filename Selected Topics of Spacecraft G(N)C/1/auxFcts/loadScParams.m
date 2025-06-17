function [SC,SEN,ACT,REQ] = loadScParams()
%% Spacecraft
SC.J = [1200, 100, -100;...
					100, 1500, 50;...
				 -100, 50, 500];	% kg m^2
SC.J = diag(diag(SC.J));
%% STR
SEN.strStd_U = [2.4; 2.4; 20] * 1/3 * (1/3600) * (pi/180); % [rad]
SEN.strDt = 0.1;		% [s]

%% Gyro
SEN.gyroArwCoeff = 0.0016 * (1/60) * (pi/180);	% [rad/sqrt(s)]
SEN.gyroFlickerCoeff = 1/3 * 0.01 * (1/3600) * (pi/180);	% [rad/s]

%% RW
ACT.rwStd = 1*1e-3;	% Assumption: 1mNm 1sigma
ACT.rwFreq = 10;	% Assumption: 10 Hz

%% REQ
REQ.eAKEreq = 1 * (1/3600) * (pi/180);	% [rad] 1 arcsec
REQ.eRKEreq = 0.5 * (1/3600) * (pi/180);	% [rad] 0.5 arcsec
REQ.rkeWindowDt = 5;	% [s]
REQ.eAPEreq = 2 * (1/3600) * (pi/180);	% [rad] 2 arcsec
REQ.eRPEreq = 1 * (1/3600) * (pi/180);	% [rad] 1 arcsec
REQ.rpeWindowDt = 5;		% [s]

REQ.uMax = 0.01;	% [Nm] tuning parameter --> maximum control effort 'jittering' due to noise
end