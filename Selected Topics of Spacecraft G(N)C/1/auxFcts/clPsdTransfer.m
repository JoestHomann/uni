function [Se,Su,figClPsds] = clPsdTransfer(G,K,Sd,Sn,fVec,fUnit,figName)
% [Se,Su,figClPsds] = clPsdTransfer(G,K,Sd,Sn,fVec,fUnit,figName);
% 
if issystem(G)
	Gmag = abs(squeeze(freqresp(G,fVec,fUnit)));
else
	Gmag = G;
end
if issystem(K)
	Kmag = abs(squeeze(freqresp(K,fVec,fUnit)));
end
Smag = 1./(1 + Gmag.*Kmag);
Tmag = Gmag.*Kmag.*Smag;

Sed = (Gmag.*Smag).^2 .* Sd;
Sen = Tmag.^2 .* Sn;
Sud = Tmag.^2 .* Sd;
Sun = Kmag.*Smag .* Sn;

Se = Sed + Sen;
Su = Sud + Sun;

%%
figClPsds = figure('Name',[figName,': PSD Transfer']);
subplot(2,1,1)
ped = loglog(fVec,Sed);
hold on
pen = loglog(fVec,Sen);
pe = loglog(fVec,Se);

grid on
legend([ped,pen,pe],{'d\rightarrowe' 'n\rightarrowe' 'S_e'},'Location','Best');
xlabel('f [Hz]');
ylabel('S_e [rad^2/Hz^{1/2}]')

subplot(2,1,2)
pud = loglog(fVec,Sud);
hold on
pun = loglog(fVec,Sun);
pu = loglog(fVec,Su);

grid on
legend([pud,pun,pu],{'d\rightarrowu' 'n\rightarrowu' 'S_u'},'Location','Best');
xlabel('f [Hz]');
ylabel('S_u [(Nm)^2/Hz^{1/2}]')
end