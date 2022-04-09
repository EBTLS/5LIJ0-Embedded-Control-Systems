%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 3
%%%%%


%% Initialization
reference = 0.002;


%% State Space Model
% Emergency Braking System
LKAS_CS.nx = 5;

EMB_CS.A = [-520,   -220,   0,          0,          0;
            220,    -500,   -999994,    0,          2e8;
            0,      1,      0,          0,          0;
            0,      0,      66667,      -0.1667,    -1.3333e7;
            0,      0,      0,          1,          0];
       
EMB_CS.B = [1000;
           0;
           0;
           0;
           ];
       
EMB_CS.C = [0 0 0 0 1];

EMB_CS.D = 0;



% % sampling time(s)
% DR.h = 2e-3;
% DCM.h = 2 * DR.h;
% DR.tau = 0.9375e-3;
% DCM.tau=0.8e-3;
