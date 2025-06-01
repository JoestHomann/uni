clear;

% Erzeugen von 4 Reihen von Zufallszahlen
% mit zunehmender Länge 

nSample = 1000;
for i = 1:4
    random_numbers{i} = rand(nSample,1);
    generator{i} = 'twister';
    nSample = nSample * 10;
end
 
% Alle Tests
dotests(random_numbers, generator);