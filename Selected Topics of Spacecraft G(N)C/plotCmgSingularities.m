function [hGrid,cGrid] = plotCmgSingularities(singDir,gimbalAxes)

singDir = singDir / norm(singDir);

nCmgs = size(gimbalAxes,2);

epsMat = [1 -1];
for ii = 2:1:nCmgs
	epsMat = combvec(epsMat,[1 -1]);
end
nSurfs = 0.5*size(epsMat,2);

hGrid = zeros(3,nCmgs,nSurfs);
cGrid = zeros(3,nCmgs,nSurfs);
for iCmg = 1:1:nCmgs
	gimbalAxis = gimbalAxes(:,iCmg);
	csk = cross(gimbalAxis,singDir) / norm(cross(gimbalAxis,singDir));
	hsk = cross(csk,gimbalAxis);
	epsk = sign(dot(hsk,singDir));
	for iSurf = 1:1:nSurfs
		hGrid(:,iCmg,iSurf) = epsMat(iCmg,iSurf)*epsk*hsk;
		cGrid(:,iCmg,iSurf) = epsMat(iCmg,iSurf)*epsk*csk;
	end
end

%%
nPlots = ceil(nSurfs/2);
i0 = 1;
for iPlot = 1:1:nPlots
	figure('Name',['CMG Singularities Plot ',int2str(iPlot)]);
	tiledlayout('flow')
	for iSurf = i0:1:i0+1
		nexttile
		title([int2str(abs(sum(epsMat(:,iSurf)))),'h Singularity - CMG planes']);
		hold on
		h0 = [0;0;0];
		for iCmg = 1:1:nCmgs
			v1 = h0 - hGrid(:,iCmg,iSurf) - cGrid(:,iCmg,iSurf);
			v2 = h0 + hGrid(:,iCmg,iSurf) - cGrid(:,iCmg,iSurf);
			v3 = h0 + hGrid(:,iCmg,iSurf) + cGrid(:,iCmg,iSurf);
			v4 = h0 - hGrid(:,iCmg,iSurf) + cGrid(:,iCmg,iSurf);
			V = [v1, v2, v3, v4, v1];
			p = fill3(V(1,:)',V(2,:)',V(3,:)','b');
			p.FaceAlpha = 0.1;
			p.EdgeAlpha = 0.1;
			quiver3(h0(1),h0(2),h0(3),hGrid(1,iCmg,iSurf),hGrid(2,iCmg,iSurf),hGrid(3,iCmg,iSurf),'b');
			quiver3(h0(1),h0(2),h0(3),cGrid(1,iCmg,iSurf),cGrid(2,iCmg,iSurf),cGrid(3,iCmg,iSurf),'r');
			quiver3(h0(1),h0(2),h0(3),gimbalAxes(1,iCmg),gimbalAxes(2,iCmg),gimbalAxes(3,iCmg),'Color',[0.5 0 0.5]);
			quiver3(h0(1),h0(2),h0(3),singDir(1),singDir(2),singDir(3),'k');
			h0 = h0 + squeeze(hGrid(:,iCmg,iSurf));
		end
		grid on;
		set(gca,'DataAspectRatio',[1 1 1]);view(3);
		%
		nexttile
		title([int2str(abs(sum(epsMat(:,iSurf)))),'h Singularity - c-vector planes']);
		hold on
		for iCmg = 1:1:nCmgs
			quiver3(0,0,0,hGrid(1,iCmg,iSurf),hGrid(2,iCmg,iSurf),hGrid(3,iCmg,iSurf),'b');
			quiver3(0,0,0,cGrid(1,iCmg,iSurf),cGrid(2,iCmg,iSurf),cGrid(3,iCmg,iSurf),'r');
			quiver3(0,0,0,singDir(1),singDir(2),singDir(3),'k');
		end
		verts = [cGrid(:,:,iSurf),-cGrid(:,:,iSurf)];
		vertsSorted = sortClockwise(verts);
		p2 = fill3(vertsSorted(1,:),vertsSorted(2,:),vertsSorted(3,:),'r');
		p2.FaceAlpha = 0.5;
		p2.EdgeAlpha = 0.1;
		grid on
	end
	i0 = i0 + 2;
end