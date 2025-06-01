function [ rnd, state ] = lcg_randi( factor, shift, modulus, seed, n )

    %lcg_randi linearer Kongruenzgenerator 
    %   Einfache Implementierung eines LCG, der Ganzzahlen liefert
    % 
    % Ein- und Ausgabeparameter
    %   factor        Multiplikationskonstante des LCG
    %   shift         Additionskonstante des LCG
    %   modulus       Modul des LCG
    %   seed          Startwert des LCG
    %   n             Anzahl der zu erzeugenden Zufallszahlen
    %   state         letzter interner Zustand des LCG
    %   rnd           Vektor der Länge n mit erzeugten Zufallszahlen

% Umwandeln der Variablen in 64-bit-Variablen 
    factor = uint64(factor);
	shift = uint64(shift);
	modulus = uint64(modulus);
	seed = uint64(seed);
% Vorbesetzen des Ausgabevektors 
	rnd(1:n) = uint64(0);
% Loop zur Erzeugung von n Zahlen 
    state = seed;
    for i = 1:n
        state = mod( factor*state + shift, modulus);
        rnd(i) = state;
    end
end % function lcg_randi