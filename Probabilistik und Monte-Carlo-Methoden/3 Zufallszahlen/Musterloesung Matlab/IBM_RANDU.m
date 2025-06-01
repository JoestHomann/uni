close all;

% Konstanten für RANDU
generator{1} = 'RANDU';
factor    = 65539;
shift     = 0;
modulus   = 2^31;
seed = 31415;

nSample = 100000;
[random_numbers{1},seed] = lcg_randu(factor,shift,modulus,seed,nSample);
dotests(random_numbers, generator);