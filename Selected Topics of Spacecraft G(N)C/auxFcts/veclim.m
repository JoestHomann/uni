function v_limited = veclim(v,vMax)
infNorm = norm(v,'inf');
if infNorm > vMax
  v_limited = v*vMax/infNorm;
else
  v_limited = v;
end