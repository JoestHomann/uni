function x = kep2rv(kep)
   
    mu = 3.986e14; 

    a     = kep(1);
    e     = kep(2);
    incl     = kep(3);
    RAAN = kep(4);
    argOfPeri = kep(5);
    trueAn = kep(6);

    p = a * (1 - e^2);

    r_pf = (p / (1 + e * cos(trueAn))) * [cos(trueAn); sin(trueAn); 0];
    v_pf = sqrt(mu / p) * [-sin(trueAn); e + cos(trueAn); 0];


    R = [ cos(RAAN)*cos(argOfPeri) - sin(RAAN)*sin(argOfPeri)*cos(incl),  -cos(RAAN)*sin(argOfPeri) - sin(RAAN)*cos(argOfPeri)*cos(incl),  sin(RAAN)*sin(incl);
          sin(RAAN)*cos(argOfPeri) + cos(RAAN)*sin(argOfPeri)*cos(incl),  -sin(RAAN)*sin(argOfPeri) + cos(RAAN)*cos(argOfPeri)*cos(incl), -cos(RAAN)*sin(incl);
          sin(argOfPeri)*sin(incl),                      cos(argOfPeri)*sin(incl),                       cos(incl) ];

    r_eci = R * r_pf;
    v_eci = R * v_pf;

    x = [r_eci; v_eci];
end