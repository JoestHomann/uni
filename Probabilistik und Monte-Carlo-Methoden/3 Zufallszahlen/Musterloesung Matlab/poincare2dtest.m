function poincare2dtest(h_axes, generator, samples)

    %poincare2dtest Test auf geometrische Korrelation in 2D 
    %   Überprüfung einer Reihe von Zufallszahlen auf geometrischen 
    %   Korrealtionen durch Plotten von Paaren von Zufallszahlen im 
    %   Einheitsquadrat
    %
    % Ein- und Ausgabeparameter
    %   h_axes     Handle zum Zeichnen von Diagrammen 
    %   generator  Name des Zufallszahlen-Generators  
    %   samples    Array von Zufallszahlen zwischen 0 und 1  

    % bestimme Anzahl der Zufallszahlen
    nSample = length(samples);
    
    version = 1;

    if version == 1
        % 1. Variante: Jeder Wert mit dem darauf folgenden Wert
        x = samples(1:nSample-1);
        y = samples(2:nSample);
    else
        % 2. Variante: Jeweils 2 Werte bilden ein Paar
        nSample = floor(nSample/2)*2;
        x = samples(1:2:nSample-1);
        y = samples(2:2:nSample);
    end
    
    % Scatterplot der Punkte
    scatter(h_axes, x, y, 0.5);
    
    % Ausgabe des Titels
    info = [ '2D-Test  mit ' num2str(nSample) ' Proben erzeut mit ' generator];
    title(h_axes, info, 'FontSize', 8, 'FontWeight', 'normal'); 
    
end %function poincare2dtest
