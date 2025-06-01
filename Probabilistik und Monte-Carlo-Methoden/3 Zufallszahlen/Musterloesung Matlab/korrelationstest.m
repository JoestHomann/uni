function korrelationstest(h_axes, generator, samples) 

    %korrelationstest Tests auf stochastische Unabhängigkeit 
    %   Überprüfung einer Reihe von Zufallszahlen auf stochastische 
    %   Unabhängigkeit anhand des zentralen Grenzwertsatzes der Statistik  
    %   mittels grafischem Vergleich und statistischen Anpassungstest 
    %   mit Chi2 und KS-Methode
    %
    % Ein- und Ausgabeparameter
    %   h_axes     Handle zum Zeichnen von Diagrammen 
    %   generator  Name des Zufallszahlen-Generators  
    %   samples    Array von Zufallszahlen zwischen 0 und 1  

    % bestimme Anzahl der Zufallszahlen
    nSample = length(samples);
    
    % lege fest, ob gegen Standarnormalverteilung getestet werden soll
    normalize = true;
    
    % Festlegung der Anzahl der zu bildenden Mittelwerte: 
    %  
    % Die Länge und Zahl der Blöcke sollen ungefähr gleich sein.
    % Wir setzen deshalb die Anzahl der Blöcke kBlock
    % (= Zahl unabhängiger Stichproben für den Mittelwert)  
    % gleich dem größtmöglichen Teiler von nSample, 
    % der kleiner gleich sqrt(nSample) ist 
    kBlock = round(sqrt(nSample));
    while true
        if mod(nSample, kBlock) == 0, break; end 
        kBlock = kBlock-1;
    end
    
    % Länge der Blöcke (= Zahl der Zufallszahlen, die für einen 
    % Mittelwert aufsummiert werden)  
    nBlock = nSample / kBlock;
    
    % Erwartungswert der Gleichverteilung
    mu = 0.5;
    
    % Standardabweichung der Gleichverteilung
    sigma = sqrt(1/12);
    
    % Berechnung der Blocksummen:
    % Forme hierzu Eingabevektor zuerst in eine Matrix mit 
    % nBlock Zeilen und kBlock Spalten um
    s = reshape(samples, nBlock, kBlock);
    % Berechne für jede der kBlock Spalten die Summmen ihrer nBlock Zeilen 
    s = sum(s,1);
    
    % Berechnung der Blockmittelwerte
    x_bar = s / nBlock;

    if normalize
        % Normalisierung der Blockmittelwerte
        x_bar_star = (x_bar - mu) / (sigma / sqrt(nBlock));
        % Berechnung der empirischen Verteilungsfunktion
        % mit normalisierten Blockmittelwerten
        [f, x] = ecdf(x_bar_star);
        % Erzeugung einer Standardnormalverteilung, mit der die Verteilung 
        % der normalisierten Blockmittelwerte verglichen wird
        nd = makedist('Normal', 'mu', 0, 'sigma', 1);
        % Erzeugen von x- und y-Werte zum Plotten der Verteilungsfunktion
        xn = linspace(-3,3,500);
        fn = cdf(nd, xn);
    else
        % Berechnung der empirischen Verteilungsfunktion
        % mit nicht normalisierten Blockmittelwerten
        [f, x] = ecdf(x_bar);
        % Erzeugung einer Normalverteilung, mit der die Verteilung 
        % der nicht normalisierten Blockmittelwerte verglichen wird
        nd = makedist('Normal', 'mu', mu, 'sigma', sigma/sqrt(nBlock));
        % Erzeugen von x- und y-Werte zum Plotten der Verteilungsfunktion
        xn = linspace(min(x),max(x),500);
        fn = cdf(nd, xn);
    end
    
    % lege Signifikanz-Level for Statistiktests fest
    alpha = 0.05;
    
    % lege Anzahl der Kategorien für Chi-Quadrat-Test fest
    nBins = max(floor(kBlock/100),4);
    
    % berechne Größen des Chi-Quadrat-Anpassungstests
    % mit normalisierten Blockmittelwerten
    if normalize
        [chi2_failed, chi2_p, chi2_stat] = chi2gof(x_bar_star, 'CDF', nd, 'NBins', nBins, 'Alpha', alpha);
    else
    % mit nicht normalisierten Blockmittelwerten
        [chi2_failed, chi2_p, chi2_stat] = chi2gof(x_bar, 'CDF', nd, 'NBins', nBins, 'Alpha', alpha);
    end
    if chi2_failed, chi2_status = 'nicht bestanden'; else, chi2_status = 'bestanden'; end
    % berechne den kritischen Wert für Chi2 
    % (das 1-alpha)-Quantil, d.h. die Inverse der Verteilungsfunktion für 1-alpha) 
    chi2_crit = chi2inv(1-alpha, chi2_stat.df);
    
    % berechne Größen des Kolmogorov-Smirnov-Anpassungstest
    if normalize
        % mit normalisierten Blockmittelwerten
        [ks_failed, ks_p, ks_stat, ks_crit] = kstest(x_bar_star, 'CDF', nd, 'Alpha', alpha);
    else
        % mit nicht normalisierten Blockmittelwerten
        [ks_failed, ks_p, ks_stat, ks_crit] = kstest(x_bar, 'CDF', nd, 'Alpha', alpha);
    end
    if ks_failed, ks_status = 'nicht bestanden'; else, ks_status = 'bestanden'; end
    
    % Zeichne Vergleichsplot der empirischen und theoretischen Verteilungsfunktionen 
    plot(x, f, xn, fn);
    
    % gebe Ergebnisse der statistischen Tests als Titel aus
    info{1} = [ 'Korrelationstest mit ' num2str(kBlock) ' Bloecken zu je ' ...
        num2str(nBlock) ' Proben erzeugt mit ' generator];
    info{end+1} = ['chi2-Test ' chi2_status ' mit X^2 = ' num2str(chi2_stat.chi2stat) ...
        ' und p(X^2) = ' num2str(chi2_p*100) '%'];
    info{end+1} = ['(krit. Wert X^2 = ' num2str(chi2_crit,'%.3g') ...
        ' fuer alpha = ' num2str(alpha*100,'%.3g') '%)'];
    info{end+1} = ['KS-Test ' ks_status ' mit d_{max} = ' num2str(ks_stat) ...
        ' und p(d_{max}) = ' num2str(ks_p*100) '%'];
    info{end+1} = ['(krit. Wert d_{max} = ' num2str(ks_crit) ...
        ' fuer alpha = ' num2str(alpha*100) '%)'];
    title(h_axes, info, 'FontSize', 8, 'FontWeight', 'normal'); 
    
end