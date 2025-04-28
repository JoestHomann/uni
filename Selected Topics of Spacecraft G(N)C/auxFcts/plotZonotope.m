function figZono = plotZonotope(Zono,varargin)
% figZono = plotZonotope(Zono[,PlotProperties])
% 
% Zono has to be a structure created by the zonotope function.
% PlotProperties is an optional structure containing e.g. xlabel,
% figure title, ...

%% Input Data Parsing
if nargin>1
	PlotProps = varargin{1};
else
	PlotProps = struct;
end

if isfield(PlotProps,'labelX')
	labelX = PlotProps.labelX;
else
	labelX = 'x';
end
if isfield(PlotProps,'labelY')
	labelY = PlotProps.labelY;
else
	labelY = 'y';
end
if isfield(PlotProps,'labelZ')
	labelZ = PlotProps.labelZ;
else
	labelZ = 'z';
end
if isfield(PlotProps,'figTitle')
	figTitle = PlotProps.figTitle;
else
	figTitle = 'Zonotope';
end

%%
figZono = figure('Name',figTitle);

if ~isempty(Zono.doi)
	quiver3(0,0,0,Zono.doi(1),Zono.doi(2),Zono.doi(3),'Color',[1 .5 0],'LineWidth',1.5);
	hold on;
end

for ii = 1:1:Zono.nFacets
	Facet = eval(['Zono.Facet',int2str(ii)]);
	p = fill3(Zono.vertexCands(Facet.iVerts(:),1),Zono.vertexCands(Facet.iVerts(:),2),Zono.vertexCands(Facet.iVerts(:),3),'b');
	if ~ishold, hold on, end
	set(p,'FaceAlpha',0.1,'EdgeColor','k','EdgeAlpha',0.1);
	if ~isempty(Zono.doi)
		if ii == Zono.iFacetDoi
			set(p,'FaceColor','y','FaceAlpha',.5);
		end
	else
		set(p,'FaceColor','c','FaceAlpha',.1);
	end
end
if ~isempty(Zono.doi)
	plot3(Zono.doi(1)/Zono.trqNorm,Zono.doi(2)/Zono.trqNorm,Zono.doi(3)/Zono.trqNorm,...
		'LineStyle','None','Marker','.','MarkerEdgeColor',[1 0.5 0]);
end
xlabel(labelX);ylabel(labelY);zlabel(labelZ);
grid on
end