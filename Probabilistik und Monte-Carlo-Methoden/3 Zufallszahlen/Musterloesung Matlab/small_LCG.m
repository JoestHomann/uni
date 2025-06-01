clear;

% Konstanten für kleinen 137-187-16
generator{1} = '137-187-16';
factor    = 137;
shift     = 187;
modulus   = 2^16;
seed = 31415;

nSample = 100000;
[random_numbers{1},seed] = lcg_randu(factor,shift,modulus,seed,nSample);
dotests(random_numbers, generator);