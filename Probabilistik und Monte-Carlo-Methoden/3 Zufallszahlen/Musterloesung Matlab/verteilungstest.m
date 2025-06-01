function verteilungstest(h_axes, generator, samples)

    %verteilungstest Tests auf Gleichverteilung 
    %   Überprüfung einer Reihe von Zufallszahlen auf Übereinstimmung 
    %   mit eine Gleichverteilung zwischen 0 und 1 mittels grafischem
    %   Vergleich und statistischen Anpassungstest mit Chi2 und KS-Methode
    %
    % Ein- und Ausgabeparameter
    %   h_axes     Handle zum Zeichnen von Diagrammen 
    %   generator  Name des Zufallszahlen-Generators  
    %   samples    Array von Zufallszahlen zwischen 0 und 1  

    % bestimme Anzahl der Zufallszahlen
    nSample = length(samples);
    
    % lege Anzahl der Balken für Histogramm und 
    % Anzahl der Kategorien für Chi-Quadrat fest
    nBins = 100;

    % erstelle Gleichverteilung mit Standardparametern für statistische
    % Tests
    ud = makedist('Uniform');
    
    % lege Signifikanz-Level for Statistiktests fest
    alpha = 0.05;
    
    % berechne Größen des Chi-Quadrat-Anpassungstests
    [chi2_failed, chi2_p, chi2_stat] = chi2gof(samples, 'CDF', ud, 'NBins', nBins, 'Alpha', alpha);
    if chi2_failed, chi2_result = 'nicht bestanden'; else, chi2_result = 'bestanden'; end
    % berechne den kritischen Wert für Chi2 
    % (das 1-alpha)-Quantil, d.h. die Inverse der Verteilungsfunktion für 1-alpha) 
    chi2_crit = chi2inv(1-alpha, chi2_stat.df);
    
    % berechne Größen des Kolmogorov-Smirnov-Anpassungstest
    [ks_failed, ks_p, ks_stat, ks_crit] = kstest(samples, 'CDF', ud, 'Alpha', alpha);
    if ks_failed, ks_result = 'nicht bestanden'; else, ks_result = 'bestanden'; end
    
    % Zeichne das Histogramm der Zufallszahlen mit nBins Balken
    histogram(h_axes, samples, nBins, 'Normalization', 'pdf', ...
        'FaceColor', [0 0.5 0.5], 'EdgeColor', 'w');
    
    % gebe Ergebnisse der statistischen Tests als Titel aus
    info{1} = [ 'Verteilungstest mit ' num2str(nSample) ...
        ' Proben erzeut mit ' generator];
    info{end+1} = ['chi2-Test ' chi2_result ' mit X^2 = ' num2str(chi2_stat.chi2stat) ...
        ' und p(X^2) = ' num2str(chi2_p*100) '%'];
    info{end+1} = ['(krit. Wert X^2 = ' num2str(chi2_crit) ' fuer alpha = ' num2str(alpha*100) '%)'];
    info{end+1} = ['KS-Test ' ks_result ' mit d_{max} = ' num2str(ks_stat) ...
        ' und p(d_{max}) = ' num2str(ks_p*100) '%'];
    info{end+1} = ['(krit. Wert d_{max} = ' num2str(ks_crit) ' fuer alpha = ' num2str(alpha*100) '%)'];
    title(h_axes, info, 'FontSize', 8, 'FontWeight', 'normal'); 
    
end % function verteilungstest