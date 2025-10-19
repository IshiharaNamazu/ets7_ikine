function qd = calc_qd(LP, SV, L, num_e, v)

%addpath(genpath("./uchidasan"))
addpath(genpath("./SpaceDyn/src/matlab/spacedyn_v2r1"))

[Gj, Addtherm] = calc_gj11( LP, SV, zeros(6,1), 1, calc_hh( LP, SV ) );

qd = pinv(Gj)*(v - Addtherm);

%endif