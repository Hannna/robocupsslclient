function  [soft_cfm,soft_erp] = calculateSoftCfm2( kp1, kd1 , kp2, kd2) 
	
kp = 1.0 / ( 1.0 / kp1 + 1.0 / kp2 );

kd = (  kd1 +  kd2  );

h = 0.001;

soft_cfm = 1.0 / (h * kp + kd)

soft_erp = h * kp / (h * kp + kd)