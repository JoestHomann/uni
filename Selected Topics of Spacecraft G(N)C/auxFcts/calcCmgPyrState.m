function cmgPyrState = calcCmgPyrState(gimbalAngles,skewAngle,cmgFwAngMom)
h1 = cmgFwAngMom * [-cos(skewAngle)*sin(gimbalAngles(1)); cos(gimbalAngles(1)); sin(skewAngle)*sin(gimbalAngles(1))];
h2 = cmgFwAngMom * [-cos(gimbalAngles(2)); -cos(skewAngle)*sin(gimbalAngles(2)); sin(skewAngle)*sin(gimbalAngles(2))];
h3 = cmgFwAngMom * [cos(skewAngle)*sin(gimbalAngles(3)); -cos(gimbalAngles(3)); sin(skewAngle)*sin(gimbalAngles(3))];
h4 = cmgFwAngMom * [cos(gimbalAngles(4)); cos(skewAngle)*sin(gimbalAngles(4)); sin(skewAngle)*sin(gimbalAngles(4))];
hCmg = h1+h2+h3+h4;

jacob = cmgFwAngMom * [...
  -cos(skewAngle)*cos(gimbalAngles(1)), sin(gimbalAngles(2)), cos(skewAngle)*cos(gimbalAngles(3)), -sin(gimbalAngles(4));...
  -sin(gimbalAngles(1)), -cos(skewAngle)*cos(gimbalAngles(2)), sin(gimbalAngles(3)), cos(skewAngle)*cos(gimbalAngles(4));...
  sin(skewAngle)*cos(gimbalAngles(1)), sin(skewAngle)*cos(gimbalAngles(2)), sin(skewAngle)*cos(gimbalAngles(3)), sin(skewAngle)*cos(gimbalAngles(4))...
  ];
%%
cmgPyrState.hCmg = hCmg;
cmgPyrState.jacob = jacob;
cmgPyrState.h1 = h1;
cmgPyrState.h2 = h2;
cmgPyrState.h3 = h3;
cmgPyrState.h4 = h4;