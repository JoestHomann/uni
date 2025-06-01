function [ rnd, state ] = lcg_randu( factor, shift, modulus, seed, n )

    %lcg_randu linearer Kongruenzgenerator für Fließkommazahlen zwischen 0 und 1
    %   Einfache Implementierung, nutzt lcg_randi
    % 
    % Ein- und Ausgabeparameter
    %   factor        Multiplikationskonstante des LCG
    %   shift         Additionskonstante des LCG
    %   modulus       Modul des LCG
    %   seed          Startwert des LCG
    %   n             Anzahl der zu erzeugenden Zufallszahlen
    %   state         letzter interner Zustand des LCG
    %   rnd           Vektor der Länge n mit erzeugten Zufallszahlen

% Berechnen von n ganzzahligen Zufallszahlen mit lcg_randi
    [rndi, state] = lcg_randi( factor, shift, modulus, seed, n );   
% Umwandeln der ganzzahligen Zufallszahlen in Gleitkommazahlen zwischen 0
% und 1
    rnd = double(rndi) / double(modulus);
   
end % function lcg_randu