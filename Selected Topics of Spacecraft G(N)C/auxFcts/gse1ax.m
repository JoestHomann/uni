function [F1,F2] = gse1ax(fFusion,varargin)
% 
% [F1,F2] = gse1ax(fFusion[,damping])
% 
% Computes single-axis gyro-stellar estimator (GSE) for a given fusion
% frequency fFusion.
% 
% The GSE filters a star tracker measurement phiMeas through a low pass
% filter F1 and an integrated gyro measurement omegaMeas/s through a high
% pass filter F2:
%								k1*s - k2											s^2						omegaMeas
% phiEst = ----------------- phiMeas + ----------------- * ----------
%						s^2 + k1*s - k2							s^2 + k1*s - k2					s
% 
% At the fusion frequency, both filters have the same magnitude. The second
% argument is an optional damping coefficient (default: 1/sqrt(2)).
% 
% (C) Ramin Geshnizjani, 2023-01-13

if nargin>2
	damping = varargin{1};
else
	damping = 1/sqrt(2);
end
om0 = 2*pi*fFusion/(sqrt(1 + sqrt(2)));	% [rad/s]
k1 = 2*damping*om0;
k2 = -om0^2;

F1 = tf([k1 -k2],[1 k1 -k2]);
F2 = tf([1 0 0],[1 k1 -k2]);