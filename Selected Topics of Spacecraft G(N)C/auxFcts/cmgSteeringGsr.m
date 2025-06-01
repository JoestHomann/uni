function gimbalRates = cmgSteeringGsr(cmgJacob,trqCmd,cmgFwAngMom,gsrPars)

jacobDetNorm = (1/cmgFwAngMom^6) * det(cmgJacob*cmgJacob');

lambda = gsrPars.a*exp(-gsrPars.b^2 * jacobDetNorm);
e1 = gsrPars.e0*sin(gsrPars.omega * gsrPars.t + gsrPars.phi1);
e2 = gsrPars.e0*sin(gsrPars.omega * gsrPars.t + gsrPars.phi2);
e3 = gsrPars.e0*sin(gsrPars.omega * gsrPars.t + gsrPars.phi3);

E = [1, e3, e2;...
  e3, 1, e1;...
  e2, e1, 1];

gimbalRates = cmgJacob' * inv(cmgJacob*cmgJacob' + cmgFwAngMom^2 * lambda*E) * trqCmd;