function plotCmgArraySingSurfs(cmgGimbalAxes,varargin)
% plotCmgArraySingSurfs(cmgGimbalAxes,cmgFwAngMom)
% Compute singular surfaces for arbitrary 4-CMG array
% Requires the 3x4 matrix cmgGimbalAxes with the gimbal axes. cmgFwAngMom
% is an optional scalar input to specify the angular momentum of all CMG
% flywheels.

cmgFwAngMom = 1;
if nargin>1
	cmgFwAngMom = varargin{1};
end

nCmgs = size(cmgGimbalAxes,2);
if nCmgs~=4
	error('Works only for four CMGs. cmgGimbalAxes must be 3x4.')
end

%% Grid for Singular Directions
gridStep = 3; % [deg]

elev = 0:gridStep:180;
azim = 0:gridStep:360;

nEl = length(elev);
nAz = length(azim);

uGrid = zeros(nAz,nEl,3);
h0Grid = zeros(nAz,nEl,3);	% for eps = {+,+,+,+}
h1Grid = zeros(nAz,nEl,3);	% for eps = {-,+,+,+}
h2Grid = zeros(nAz,nEl,3);	% for eps = {+,-,+,+}
h3Grid = zeros(nAz,nEl,3);	% for eps = {+,+,-,+}
h4Grid = zeros(nAz,nEl,3);	% for eps = {+,+,+,-}
h5Grid = zeros(nAz,nEl,3);	% for eps = {-,-,+,+}
h6Grid = zeros(nAz,nEl,3);	% for eps = {-,+,-,+}
h15Grid = zeros(nAz,nEl,3);	% for eps = {-,-,-,-}

eps0 = [1,1,1,1];
eps1 = [-1,1,1,1];
eps2 = [1,-1,1,1];
eps3 = [1,1,-1,1];
eps4 = [1,1,1,-1];
eps5 = [-1,-1,1,1];
eps6 = [-1,1,-1,1];
eps15= [-1,-1,-1,-1];

for iAz=1:1:nAz
	for iEl = 1:1:nEl
		ux = sind(azim(iAz));
		uy = cosd(azim(iAz))*cosd(elev(iEl));
		uz=cosd(azim(iAz))*sind(elev(iEl));
		singDir = [ux;uy;uz];

		uGrid(iAz,iEl,:) = [ux;uy;uz];
		for iCmg = 1:1:nCmgs
			gimbalAxis = cmgGimbalAxes(:,iCmg);
			gxu = cross(gimbalAxis,singDir);
			hsk = cmgFwAngMom * cross(gxu,gimbalAxis) / norm(gxu);
			epsk = sign(dot(hsk,singDir));
			h0Grid(iAz,iEl,:) = squeeze(h0Grid(iAz,iEl,:)) + eps0(iCmg)*epsk*hsk;
			h1Grid(iAz,iEl,:) = squeeze(h1Grid(iAz,iEl,:)) + eps1(iCmg)*epsk*hsk;
			h2Grid(iAz,iEl,:) = squeeze(h2Grid(iAz,iEl,:)) + eps2(iCmg)*epsk*hsk;
			h3Grid(iAz,iEl,:) = squeeze(h3Grid(iAz,iEl,:)) + eps3(iCmg)*epsk*hsk;
			h4Grid(iAz,iEl,:) = squeeze(h4Grid(iAz,iEl,:)) + eps4(iCmg)*epsk*hsk;
			h5Grid(iAz,iEl,:) = squeeze(h5Grid(iAz,iEl,:)) + eps5(iCmg)*epsk*hsk;
			h6Grid(iAz,iEl,:) = squeeze(h6Grid(iAz,iEl,:)) + eps6(iCmg)*epsk*hsk;
			h15Grid(iAz,iEl,:) = squeeze(h15Grid(iAz,iEl,:)) + eps15(iCmg)*epsk*hsk;
		end
	end
end

%% Plot

colorFace = [.0 .5 .8];
colorEdge = [0 .5 .8];
% envelope: {+,+,+,+}
figure('Name','Envelope')
hold on
surf(h0Grid(:,:,1),h0Grid(:,:,2),h0Grid(:,:,3),'FaceColor',colorFace,'FaceAlpha',0.3,'EdgeColor',colorEdge,'EdgeAlpha',0.1);
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
set(gca,'DataAspectRatio',[1 1 1]);view(3);


figure('Name','2h Singularity -+++')
hold on
surf(h1Grid(:,:,1),h1Grid(:,:,2),h1Grid(:,:,3),'FaceColor',colorFace,'FaceAlpha',0.3,'EdgeColor',colorEdge,'EdgeAlpha',0.1);
title('2h Singularity {-,+,+,+}')
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
set(gca,'DataAspectRatio',[1 1 1]);view(3);

figure('Name','2h Singularity +-++')
hold on
surf(h2Grid(:,:,1),h2Grid(:,:,2),h2Grid(:,:,3),'FaceColor',colorFace,'FaceAlpha',0.3,'EdgeColor',colorEdge,'EdgeAlpha',0.1);
title('2h Singularity {+,-,+,+}')
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
set(gca,'DataAspectRatio',[1 1 1]);view(3);

figure('Name','2h Singularity ++-+')
hold on
surf(h3Grid(:,:,1),h3Grid(:,:,2),h3Grid(:,:,3),'FaceColor',colorFace,'FaceAlpha',0.3,'EdgeColor',colorEdge,'EdgeAlpha',0.1);
title('2h Singularity {+,+,-,+}')
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
set(gca,'DataAspectRatio',[1 1 1]);view(3);

figure('Name','2h Singularity +++-')
hold on
surf(h4Grid(:,:,1),h4Grid(:,:,2),h4Grid(:,:,3),'FaceColor',colorFace,'FaceAlpha',0.3,'EdgeColor',colorEdge,'EdgeAlpha',0.1);
title('2h Singularity {+,+,+,-}')
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
set(gca,'DataAspectRatio',[1 1 1]);view(3);

figure('Name','0h Singularity --++')
hold on
surf(h5Grid(:,:,1),h5Grid(:,:,2),h5Grid(:,:,3),'FaceColor',colorFace,'FaceAlpha',0.3,'EdgeColor',colorEdge,'EdgeAlpha',0.1);
title('0h Singularity {-,-,+,+}')
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
set(gca,'DataAspectRatio',[1 1 1]);view(3);

figure('Name','0h Singularity -+-+')
hold on
surf(h6Grid(:,:,1),h6Grid(:,:,2),h6Grid(:,:,3),'FaceColor',colorFace,'FaceAlpha',0.3,'EdgeColor',colorEdge,'EdgeAlpha',0.1);
title('0h Singularity {-,+,-,+}')
grid on;
xlabel('x');
ylabel('y');
zlabel('z');
set(gca,'DataAspectRatio',[1 1 1]);view(3);

% figure('Name','neg. Envelope')
% hold on
% surf(h15Grid(:,:,1),h15Grid(:,:,2),h15Grid(:,:,3),'FaceColor',colorFace,'FaceAlpha',0.3,'EdgeColor',colorEdge,'EdgeAlpha',0.1);
% grid on
% set(gca,'DataAspectRatio',[1 1 1]);view(3);