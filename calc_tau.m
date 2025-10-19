function tau = calc_tau(LP, SV, qdd)

[H_asuta, C_asuta] = calc_asuta(LP, SV);

tau = H_asuta * qdd + C_asuta;

end