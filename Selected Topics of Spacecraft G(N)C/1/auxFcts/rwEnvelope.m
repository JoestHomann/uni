function Zono = rwEnvelope(A,hwMax,varargin)

%% Input Data Parsing
p = inputParser;

checkA = @(x) isnumeric(x) && size(x,1) == 3;
checkVec = @(x) isnumeric(x) && size(x,1) == 3 && size(x,2) == 1;

addRequired(p,'A',checkA);
addRequired(p,'hwMax',@isnumeric);
addOptional(p,'doi',[],checkVec);

parse(p,A,hwMax,varargin{:});

doi  = p.Results.doi;

% if size(A,1)~=3
% 	error ('Please provide a 3xn configuration matrix.');
% end
if size(hwMax,2)~=1 || ~(size(hwMax,1)==1 || size(hwMax,1)==size(A,2))
	error ('Please provide either a scalar angMom/torque magnitude per wheel or an nx1 vector of angMom/torque magnitudes.')
end
% 
% if nargin>2
% 	if ~all(size(varargin{1}) = [3,1])
% 		error('Optional direction or angMom/trq command must be 3x1.');
% 	end
% 	doi = varargin{1};	% direction of interest
% else
% 	doi = [];
% end

%% Preliminaries
nWheels = size(A,2);	% # of wheels
nVertsMax = nWheels^2 - nWheels + 2;	% maximum number of vertices
nFacetsMax = nWheels*(nWheels-1);			% maximum number of facets

As = A*diag(hwMax);	% Scale every RW direction with its maximum angMom/trq

doiProjections = zeros(1,nFacetsMax);
%% Compute Vertex Candidates
% create 2^n combinations of {-1,1} to compute vertex candidates:
combs = [1,-1];
for ii=2:1:nWheels
	combs = combvec(combs,[1,-1]);
end

vertexCands = unique((As * combs)','rows');

%% Compute Facets
iFacet = 0;
for ii=1:1:nWheels-1
	for jj=ii+1:1:nWheels
		aiaj = cross(A(:,ii),A(:,jj));
		if norm(aiaj) > 0	% it's only a facet if ai and aj are not parallel
			iFacet = iFacet + 1;
			n = aiaj/norm(aiaj);
			D = sum(abs(As'*n));	% Distance of ij-facet = sum of | dot(a_i , n_ij)  |
			nScaled = n/D;
			
			% Find vertices on this facet (and its opposite one due to symmetry):
			vertexCandDists = round(vertexCands * n,8);
			iVertsPos = find(vertexCandDists==max(vertexCandDists));
			iVertsNeg = find(vertexCandDists==min(vertexCandDists));
      [~,iVertsPosSorted] = sortClockwise(vertexCands',iVertsPos);
      [~,iVertsNegSorted] = sortClockwise(vertexCands',iVertsNeg);
% 			iVertsPosSorted = iVertsPos;
% 			iVertsNegSorted = iVertsNeg;
			
			FacetPos.ij = [int2str(ii),int2str(jj)]; FacetNeg.ij = [int2str(jj),int2str(ii)];
      FacetPos.ai = A(:,ii); FacetNeg.ai = A(:,jj);
      FacetPos.aj = A(:,jj); FacetNeg.aj = A(:,ii);
      FacetPos.n = n; FacetNeg.n = -n;
      FacetPos.D = D; FacetNeg.D = D;
      FacetPos.nScaled = nScaled; FacetNeg.nScaled = -nScaled;
      FacetPos.iVerts = iVertsPosSorted; FacetNeg.iVerts = iVertsNegSorted;
			
			eval(['Facet',int2str(iFacet),' = FacetPos;']);
      eval(['Facet',int2str(iFacet+1),' = FacetNeg;']);
      if ~isempty(doi)
        doiProjections(iFacet) = dot(doi,nScaled);
        doiProjections(iFacet+1) = -dot(doi,nScaled);
      end
      
      iFacet = iFacet + 1;
		end
	end
end

nFacets = iFacet;
doiProjections = doiProjections(1:nFacets);
if ~isempty(doi)
  [trqNorm, iFacetDoi] = max(doiProjections);
  FacetDoi = eval(['Facet',int2str(iFacetDoi)]);
end

%% Output Data Parsing
Zono.nFacets = nFacets;
Zono.vertexCands = vertexCands;
for ii=1:1:nFacets
  eval(['Zono.Facet',int2str(ii),' = Facet',int2str(ii),';']);
end
Zono.doi = doi;
if ~isempty(doi)
  Zono.doiProjections = doiProjections;
  Zono.trqNorm = trqNorm;
  Zono.iFacetDoi = iFacetDoi;
  Zono.FacetDoi = FacetDoi;
end