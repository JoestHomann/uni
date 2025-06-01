function dotests( r, generator, varargin )

    if ~isempty(varargin)
        tests = varargin;
    else
        tests = {'Verteilung'; 'Korrelation'; '2D'; '3D'; 'Spektral'}; 
    end

    nRand = length(r);
    nCol  = floor(sqrt(nRand-1))+1;
    nRow  = floor((nRand-1)./nCol)+1;
    sizeX = nCol * 16;
    sizeY = nRow * 10;
    
    for j = 1:length(tests)
        figure;
        set(gcf,'units','centimeters', 'Position', [0 0 sizeX sizeY] );
        for i = 1:nRand
            h_axes = subplot(nRow,nCol,i);
            if strcmp(tests{j},'Verteilung')
                verteilungstest(h_axes, generator{i}, r{i});
            elseif strcmp(tests{j},'Korrelation')
                korrelationstest(h_axes, generator{i}, r{i});
            elseif strcmp(tests{j},'2D')
                poincare2dtest(h_axes, generator{i}, r{i});
            elseif strcmp(tests{j},'3D')
                poincare3dtest(h_axes, generator{i}, r{i});
            elseif strcmp(tests{j},'Spektral') 
                spektraltest(h_axes, generator{i}, r{i});
            end
        end
    end
end