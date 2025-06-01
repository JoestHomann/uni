function poincare3dtest(h_axes, generator, samples)

    %poincare3dtest Test auf geometrische Korrelation in 3D 
    %   Überprüfung einer Reihe von Zufallszahlen auf geometrischen 
    %   Korrealtionen durch Plotten von Tripeln von Zufallszahlen im 
    %   Einheitswürfel
    %
    % Ein- und Ausgabeparameter
    %   h_axes     Handle zum Zeichnen von Diagrammen 
    %   generator  Name des Zufallszahlen-Generators  
    %   samples    Array von Zufallszahlen zwischen 0 und 1  

    % bestimme Anzahl der Zufallszahlen
    nSample = length(samples);
    
    version = 2;

    if version == 1
        % 1. Variante: Jeder Wert mit den 2 darauf folgenden Werten
        x = samples(1:nSample-2);
        y = samples(2:nSample-1);
        z = samples(3:nSample);
    else
        % 2. Variante: Jeweils 3 Werte bilden einen Punkt
        nSample = floor(nSample/3)*3;
        x = samples(1:3:nSample-2);
        y = samples(2:3:nSample-1);
        z = samples(3:3:nSample);
    end
    
    % Scatterplot der Punkte
    scatter3(x, y, z, 0.5);
    
    % Ausgabe des Titels
    info = [ '3D-Test  mit ' num2str(nSample) ' Proben erzeut mit ' generator];
    title(h_axes, info, 'FontSize', 8, 'FontWeight', 'normal'); 
    
end