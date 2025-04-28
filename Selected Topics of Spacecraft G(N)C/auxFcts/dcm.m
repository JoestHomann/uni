function [ C ] = dcm(angle,axis)
% C = dcm(angle,axis)
% Generates the elementary direction cosine matrix for a rotation of angle
% radians around axis
% 
% INPUTS:
% -------
% angle     ->  Rotation angle in radians
% axis      ->  Rotation axis ('x','y','z')
% 
% OUTPUTS:
% --------
% C         ->  Direction cosine matrix
% 
% 
% (c) Ramin Geshnizjani, 2013/2019
% Flight Mechanics and Controls Lab, University of Stuttgart

switch lower(axis)
    case 'x'
        C = [1 0 0;...
             0 cos(angle) sin(angle);...
             0 -sin(angle) cos(angle)];
    case 'y'
        C = [cos(angle) 0 -sin(angle);...
             0 1 0;...
             sin(angle) 0 cos(angle)];
    case 'z'
        C = [cos(angle) sin(angle) 0;...
             -sin(angle) cos(angle) 0;...
             0 0 1];
    otherwise
        error('Axis has to be ''x'', ''y'', or ''z''.');
end

end

