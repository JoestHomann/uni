clear;

% Konstanten für CRAND
generator{1} = 'crand';
factor    = 1103515245;
shift     = 12345;
modulus   = 2^31;
seed = 1;

nSample = 10;
[r1,x] = lcg_randi(factor,shift,modulus,seed,nSample);
[r2,x] = lcg_randi_32(factor,shift,modulus,seed,nSample);
[r3,x] = lcg_randi_double(factor,shift,modulus,seed,nSample);
r1 = double(r1)/double(modulus);
r2 = double(r2)/double(modulus);
r3 = double(r3)/double(modulus);
plot(1:nSample,r1,'-ko');
hold on
plot(1:nSample,r2,'-rs');
plot(1:nSample,r3,'-gd');
hold off
legend("uint64","uint32","double");