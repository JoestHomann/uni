function [ rnd, state ] = lcg_randi_double( factor, shift, modulus, seed, n )

    %lcg_randi_double  
    %   Inkorrekte Implementierung eines LCG, der Ganzzahlen liefern
    %   soll, mit dem Matlab-Standarddatentyp "double" 
    % 
    % Ein- und Ausgabeparameter
    %   factor        Multiplikationskonstante des LCG
    %   shift         Additionskonstante des LCG
    %   modulus       Modul des LCG
    %   seed          Startwert des LCG
    %   n             Anzahl der zu erzeugenden Zufallszahlen
    %   state         letzter interner Zustand des LCG
    %   rnd           Vektor der Länge n mit erzeugten Zufallszahlen

% Vorbesetzen des Ausgabevektors 
	rnd(1:n) = 0;
% Loop zur Erzeugung von n Zahlen 
	state = seed;
    for i = 1:n
        state = mod( factor*state + shift, modulus);
        rnd(i) = state;
    end
end % function lcg_randi_double