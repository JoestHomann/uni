%% 4-CMG Roof
gimbalAxes = [...
	0,	0,	-1;	...
	0,	0,	-1;	...
	0, -1,	0; ...
	0, -1,	0; ...
	]';

%% 4-CMG Pyramid
pyrBeta = 53.13 * pi/180;
gimbalAxes = [...
	sin(pyrBeta),	0,	cos(pyrBeta);...
	0,	sin(pyrBeta),	cos(pyrBeta);...
	-sin(pyrBeta),	0,	cos(pyrBeta);...
	0,	-sin(pyrBeta),	cos(pyrBeta);...
	]';

%% Plot Singular Surfaces of Array
plotCmgArraySingSurfs(gimbalAxes);

%% Choose singular direction and plot singular config
singDir = [1;2;2];	% will be normalized in the function

[hGrid,cGrid] = plotCmgSingularities(singDir,gimbalAxes);
%% Check that singular direction is in fact orthogonal to Jacobian
nSurfs = size(cGrid,3);	% number of singular surfaces
for iSurf = 1:1:nSurfs
	C = squeeze(cGrid(:,:,iSurf));
	disp(['Projection of c-vectors on singular direction for surface ',int2str(iSurf),':'])
	singProj = C'*singDir
	if any(abs(singProj)>1e-10)
		warning('not singular?!');
	end
end