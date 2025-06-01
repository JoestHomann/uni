function [ rnd, state ] = lcg_randi_32( factor, shift, modulus, seed, n )

    %lcg_randi_32 linearer Kongruenzgenerator 
    %   Implementierung eines LCG, der Ganzzahlen liefert mit 
    %   32-bit vorzeichenlosem Datentyp
    %   Achtung: Läuft in Sättigung, wenn der interne Zustand 
    %   größer 2^32 wird
    % 
    % Ein- und Ausgabeparameter
    %   factor        Multiplikationskonstante des LCG
    %   shift         Additionskonstante des LCG
    %   modulus       Modul des LCG
    %   seed          Startwert des LCG
    %   n             Anzahl der zu erzeugenden Zufallszahlen
    %   state         letzter interner Zustand des LCG
    %   rnd           Vektor der Länge n mit erzeugten Zufallszahlen

% Umwandeln der Variablen in 32-bit-Variablen 
    factor = uint32(factor);
	shift = uint32(shift);
	modulus = uint32(modulus);
	seed = uint32(seed);
% Vorbesetzen des Ausgabevektors 
	rnd(1:n) = uint32(0);
% Loop zur Erzeugung von n Zahlen 
    state = seed;
    for i = 1:n
        state = mod( factor*state + shift, modulus);
        rnd(i) = state;
    end
end % function lcg_randi_32

