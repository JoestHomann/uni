function [vertsSorted, iVertsSorted] = sortClockwise(verts, varargin)
% [vertsSorted, varargout] = sortVertices(verts[, iVerts])
% 
% Sorts the vertices of a polygon (counter-)clockwise such that the polygon
% can be readily plotted using fill3.
% 
% INPUTS:
% -------
% verts   ->  Matrix with 3d coordinates of vertices  (3xn)
% iVerts  ->  Vector containing the indices of the vertices to be used
%             (optional)
% 
% OUTPUTS:
% --------
% vertsSorted   ->  Matrix with sorted 3d coordinates of (selected) vertices
% iVertsSorted  ->  Correspondingly sorted index vector (if provided)
% 
% (c) Ramin Geshnizjani, 2018/2019
% Flight Mechanics and Controls Lab, University of Stuttgart

if nargin>1
  iVerts = varargin{1};
else
  iVerts = 1:1:size(verts,2);
end

verts = verts(:,iVerts);
%% Compute Facet Normal and DCM to align it with the z-Axis
n = cross(verts(:,1),verts(:,2))/norm(cross(verts(:,1),verts(:,2)));

phi = asin(-n(2));
if abs(n(3))<eps
  theta = pi/2;
else
  theta = atan(n(1)/n(3));
end

Tfa = dcm(phi,'x')*dcm(theta,'y');

verts_f = Tfa * verts;

%% Compute Center of Facet and Angles to All Vertices
cEta = mean(verts_f(1,:));
cZeta = mean(verts_f(2,:));

a = atan2(verts_f(2,:)-cZeta, verts_f(1,:)-cEta);

%% Determine Order and Sort Vertices
[~,order] = sort(a,'descend');
vertsSorted_f = verts_f(:,order);
vertsSorted = Tfa'*vertsSorted_f;
iVertsSorted = iVerts(order);
end

