function [kep] = rv2kep(x)

r_vec = x(1:3);
v_vec = x(4:6);

r = norm(r_vec);
v = norm(v_vec);

h_vec = cross(r_vec, v_vec);
h = norm(h_vec);

mu = 3.986e14;
epsilon = v^2/2-mu/r;

a = - mu/(2*epsilon);

e_vec = ((v^2 - mu/r)*r_vec - dot(r_vec, v_vec)*v_vec)/mu;
e = norm(e_vec);

Z = [0; 0; 1];
n_vec = cross(Z, h_vec);
n = norm(n_vec);

incl = acos(h_vec(3)/norm(h));

if n_vec(2) > 0
    RAAN = acos(n_vec(1)/n);
elseif n_vec(2) < 0
    RAAN = 2*pi - acos(n_vec(1)/n);
else
    RAAN = 0;
end

if e_vec(3) > 0
    argOfPeri = acos(dot(n_vec, e_vec)/(n*e));
elseif e_vec(3) < 0
    argOfPeri = 2*pi - acos(dot(n_vec, e_vec)/(n*e));
else
    argOfPeri = 0;
end

if dot(r_vec, v_vec) > 0
    trueAn =acos(dot(e_vec, r_vec)/(e*r));
elseif dot(r_vec, v_vec) < 0
    trueAn = 2*pi - acos(dot(e_vec, r_vec)/(e*r));
else
    trueAn = 0;
end


kep = [a; e; incl; RAAN; argOfPeri; trueAn];