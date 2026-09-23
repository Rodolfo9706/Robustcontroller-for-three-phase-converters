%% System parameters

R = 0.1; %3 Oms
L = 100e-3; %10mH 10e-3 ---- si se cambia este hay que ajustar gains
C = 1220e-6; %2500 uF

w = 2000;
Rlnom = 33; %45 Oms

omega = w*pi; %60Hz
E = 16; % 80volts  



%% Desired values:
vod = 16.3;

%arg = E^2 - (8*R*vod^2)/(3*Rl);
%idd = E/(2*R) +  sqrt(arg)/(2*R);


%% Control parameters librarie power electronics
%k1 = 5000000;    k2 = 400000;     k3 = 700000;

%k1 =40000;  k2 =10000; k3 =30000;%1200; %60

%k1 = 50000;    k2 = 4000;     k3 = 7000;


k1 =100; k2 =50; k3 = 20;%;3000;%1200; %60

% antes del signo de omega k1 = 30;    k2 = 10;     k3 = 120;

% Adaptive control terms
am = 1/(Rlnom*C); % 1/(Rl*C) = 8.889, am = 1/(Rl*C) - Theta*
bm = 1;
kappa = 320;

lambda = 100;
Qas = bm;
Thetaas = 0;



%% SMC Control parameters
lambbda_d = 2900;
eta_d = 10;
lambda_q = 20;
eta_q = 8;

Ts = 0.002;