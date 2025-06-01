% "Test der Tests" mit stochastisch nicht unabhängigen Zufallszahlen

% Zur Sicherheit werden alle evtl. vorhandenen Abbildungen geschlossen ...
clear;  

generator{1} = 'cheat';
% erzeuge Vektor mit gleichverteilten Zufallszahlen mit 50000 Zeilen
% (mit Matlab-Defaultgenerator "twister")
a = rand(50000,1);
% erzeuge weitere 50000 Zahlen linear abhängig von a 
b = 1 - a;
% Variante 1:  50000 Zahlen aus a gefolht von 50000 Zahlen aus b
generator{1} = 'cheat1';
random_numbers{1} = [a;b];
% Variante 2: je einen Zahl aus a gefolgt von einer Zahls aus b
generator{2} = 'cheat2';
random_numbers{2}  = [a,b];
random_numbers{2}  = reshape(random_numbers{2}',100000,1);
dotests(random_numbers, generator);