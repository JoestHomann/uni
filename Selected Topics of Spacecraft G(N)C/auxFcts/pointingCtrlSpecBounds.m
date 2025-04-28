function figBounds = pointingCtrlSpecBounds(plant,noisePsd,distPsd,eReqPsd,uReqPsd)

%% Parse Input Data
if iscell(noisePsd)
	nN = length(noisePsd);
else
	nN = 1;
	noisePsd = {noisePsd};
end
if iscell(distPsd)
	nD = length(distPsd);
else
	nD = 1;
	distPsd = {distPsd};
end
if iscell(eReqPsd)
	nE = length(eReqPsd);
else
	nE = 1;
	eReqPsd = {eReqPsd};
end
if iscell(uReqPsd)
	nU = length(uReqPsd);
else
	nU = 1;
	uReqPsd = {uReqPsd};
end

magG = squeeze(get(abs(plant),'ResponseData'));
fVec = get(plant,'Frequency');
fUnit = get(plant,'FrequencyUnit');
%% Compute Specifications
% n -> e
Tspec1 = cell(nE,nN);
Tspec1name = cell(nE,nN);
for iN = 1:1:nN
	Sn = squeeze(get(abs(noisePsd{iN}),'ResponseData'));
	nameIn = get(noisePsd{iN},'OutputName');
	for iE = 1:1:nE
		Se = squeeze(get(abs(eReqPsd{iE}),'ResponseData'));
		nameOut = get(eReqPsd{iE},'OutputName');
		Tspec1{iE,iN} = sqrt(Se)./sqrt(Sn);
		Tspec1name{iE,iN} = ['T_{',nameIn{:},'\rightarrow',nameOut{:},'}'];
	end
end

% d -> e
Sspec = cell(nE,nD);
SspecName = cell(nE,nD);
for iD = 1:1:nD
	Sd = squeeze(get(abs(distPsd{iD}),'ResponseData'));
	nameIn = get(distPsd{iD},'OutputName');
	for iE = 1:1:nE
		Se = squeeze(get(abs(eReqPsd{iE}),'ResponseData'));
		nameOut = get(eReqPsd{iE},'OutputName');
		Sspec{iE,iD} = 1./magG .* sqrt(Se)./sqrt(Sd);
		SspecName{iE,iD} = ['S_{',nameIn{:},'\rightarrow',nameOut{:},'}'];
	end
end

% n -> u
Tspec2 = cell(nU,nN);
Tspec2name = cell(nU,nN);
for iN = 1:1:nN
	Sn = squeeze(get(abs(noisePsd{iN}),'ResponseData'));
	nameIn = get(noisePsd{iN},'OutputName');
	for iU = 1:1:nU
		Su = squeeze(get(abs(uReqPsd{iU}),'ResponseData'));
		nameOut = get(uReqPsd{iU},'OutputName');
		Tspec2{iU,iN} = magG .* sqrt(Su)./sqrt(Sn);
		Tspec2name{iU,iN} = ['T_{',nameIn{:},'\rightarrow',nameOut{:},'}'];
	end
end

% d -> u
Tspec3 = cell(nU,nD);
Tspec3name = cell(nU,nD);
for iD = 1:1:nD
	Sd = squeeze(get(abs(distPsd{iD}),'ResponseData'));
	nameIn = get(distPsd{iD},'OutputName');
	for iU = 1:1:nU
		Su = squeeze(get(abs(uReqPsd{iU}),'ResponseData'));
		nameOut = get(uReqPsd{iU},'OutputName');
		Tspec3{iU,iN} = sqrt(Su)./sqrt(Sd);
		Tspec3name{iU,iN} = ['T_{',nameIn{:},'\rightarrow',nameOut{:},'}'];
	end
end
%% Plot Specification Bounds
nPlots = length(Sspec)+length(Tspec1)+length(Tspec2)+length(Tspec3);
p = NaN(1,nPlots);
legStr = cell(1,nPlots);
figBounds = figure('Name','Closed Loop TF Specification Bounds');
semilogx(fVec([1 end]),[0 0],'k','DisplayName','');
hold on
jj=0;
for ii = 1:1:length(Sspec)
	jj = jj + 1;
	p(jj) = semilogx(fVec,mag2db(Sspec{ii}),'DisplayName',SspecName{ii});
	legStr{jj} = SspecName{ii};
end
for ii = 1:1:length(Tspec1)
	jj = jj+1;
	p(jj) = semilogx(fVec,mag2db(Tspec1{ii}),'DisplayName',Tspec1name{ii});
	legStr{jj} = Tspec1name{ii};
end
for ii = 1:1:length(Tspec2)
	jj = jj+1;
	p(jj) = semilogx(fVec,mag2db(Tspec2{ii}),'DisplayName',Tspec2name{ii});
	legStr{jj} = Tspec2name{ii};
end
for ii = 1:1:length(Tspec3)
	jj = jj+1;
	p(jj) = semilogx(fVec,mag2db(Tspec3{ii}),'DisplayName',Tspec3name{ii});
	legStr{jj} = Tspec3name{ii};
end

xlabel(['Frequency [',fUnit,']']);
ylabel('Magnitude [dB]');
grid on
legend(p,legStr);

end