function spektraltest(h_axes, generator, samples) 

    %spektraltest Test auf Gleichverteilung und Unabhängigkeit 
    %   einer Reihe von Zufallszahlen mittels diskreter Fourieranalyse
    %
    % Ein- und Ausgabeparameter
    %   h_axes     Handle zum Zeichnen von Diagrammen 
    %   generator  Name des Zufallszahlen-Generators  
    %   samples    Array von Zufallszahlen zwischen 0 und 1  

    % bestimme Anzahl der Zufallszahlen
    nSample = length(samples);

    % Berechnung der diskreten Fouriertransformierten der Samples 
    % (liefert die (komplexen) Amplituden der Frequenzen 
    Y = fft(samples);
    
    % Normierung und Berechnung des Betrags der (komplexen) Amplituden 
    YAbs = abs(Y/nSample);
    
    % Die Funktion fft liefert ein zweiseitiges Spektrum, 
    % d.h. die Amplituden für die positiven und die negativen Frequenzen
    % Da das Signal symmetrisch um 0 ist, wird üblicherweise nur das
    % einseitige Spektrum dargestellt 
    
    % Da außerdem die Amplitude des nullten Modus (entspricht dem Mittelwert)
    % viel größer ist als die anderen ist, soll sie nicht mit geplottet werden
    
    % Extrahieren der Amplitude des nullten Modus
    Y0 = YAbs(1);
    
    % Extrahieren des einseitigen Spektrums (positiven Hälfte des Spektrums
    % ohne nullten Modus))
    Y1 = YAbs(2:nSample/2+1);
    % Verdoppeln der Werte der positiven Hälfte zur Berücksichtigung
    % der (symmetrischen) negative Hälfte; 
    Y1 = 2*Y1;
    
    % Plotten des einseitigen Spektrums
    plot(h_axes, Y1);
    
    info{1} = [ 'Spektraltest  mit ' num2str(nSample) ' Proben erzeut mit ' generator];
    info{end+1} = [ 'Amplitude der Frequenz 0 (Mittelwert) = ' num2str(Y0)];
    title(h_axes, info, 'FontSize', 8, 'FontWeight', 'normal'); 
    
end