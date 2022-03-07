%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 1
%%% Author: Jiaxuan Zhang
%%%%%

%% Parameters
% Dual Rotary System Parameters
DR.Km = 4.4 * 1e-2;
DR.J1 = 3.75 * 1e-6; DR.J2 = 3.75 * 1e-6;
DR.b = 1 * 1e-5;
DR.k = 0.2656;
DR.d = 3.125 * 1e-5;
DR.nx = 4;
DR.nu = 1;

% DC Motor Speed Control System Parameters
DCM.K = 0.01;
DCM.J = 0.01;
DCM.b = 0.1;
DCM.R = 1;
DCM.L = 0.5;
DCM.nx = 2;
DCM.nu = 1;

% sampling time(s)
DR.h = 0.2e-3;
DCM.h = 2 * DR.h;
tau = 0.3e-3;

%% State Space Model
% Dual Rotary System
DR_CS.A = [0, 0, 1, 0;
        0, 0, 0, 1;
        -DR.k / DR.J1, DR.k / DR.J1, - (DR.d + DR.b) / DR.J1, (DR.d + DR.b) / DR.J1;
        -DR.k / DR.J2, DR.k / DR.J2, (DR.d + DR.b) / DR.J2, - (DR.d + DR.b) / DR.J2];
DR_CS.B = [0;
        0;
        DR.Km / DR.J1;
        0];
DR_CS.C = [1 1 0 0];

% DC Motor Speed Control System
DCM_CS.A = [-DCM.b / DCM.J, DCM.K / DCM.J;
        -DCM.K / DCM.L, DCM.R / DCM.L];
DCM_CS.B = [0;
        1 / DCM.L];
DCM_CS.C = [1 0];

%% Discrete-time model without delay
% DR Control System
% continuous-time
DR_CS.sysc = ss(DR_CS.A, DR_CS.B, DR_CS.C, 0);

% to discrete
DR_CS.sysd = c2d(DR_CS.sysc, DR.h);
DR_CS.phi = DR_CS.sysd.a; DR_CS.Gamma = DR_CS.sysd.b; DR_CS.Cd = DR_CS.sysd.c;
% Desired closed-loop poles and pole placement
DR_CS.alpha = [0.95 0.98 0.98 0.98];
% feedback vector
DR_CS.K = -acker(DR_CS.phi, DR_CS.Gamma, DR_CS.alpha);
% feedforward gain
temp = inv(eye(4) - DR_CS.phi - DR_CS.Gamma * DR_CS.K) * DR_CS.Gamma;
DR_CS.F = 1 / (DR_CS.C * temp);

DR_CS.full_sysc = ss(DR_CS.A + DR_CS.B * DR_CS.K, DR_CS.B * DR_CS.F, DR_CS.C, 0);
DR_CS.full_sysd = ss(DR_CS.full_sysc, DR.h);
DR_CS.full_tf = tf(DR_CS.full_sysd);

% DCM Control System
% continuous-time
DCM_CS.sysc = ss(DCM_CS.A, DCM_CS.B, DCM_CS.C, 0);

% to discrete
DCM_CS.sysd = c2d(DCM_CS.sysc, DCM.h);
DCM_CS.phi = DCM_CS.sysd.a; DCM_CS.Gamma = DCM_CS.sysd.b; DCM_CS.Cd = DCM_CS.sysd.c;

% Desired closed-loop poles and pole placement
DCM_CS.alpha = [0.998 0.998];

% feedback vector
DCM_CS.K = -acker(DCM_CS.phi, DCM_CS.Gamma, DCM_CS.alpha);

% feedforward gain
temp = inv(eye(2) - DCM_CS.phi - DCM_CS.Gamma * DCM_CS.K) * DCM_CS.Gamma;
DCM_CS.F = 1 / (DCM_CS.C * temp);

DCM_CS.full_sysc = ss(DCM_CS.A + DCM_CS.B * DCM_CS.K, DCM_CS.B * DCM_CS.F, DCM_CS.C, 0);
DCM_CS.full_sysd = ss(DCM_CS.full_sysc, DCM.h);
DCM_CS.full_tf = tf(DCM_CS.full_sysd);

% Review Controller Result
figure
DR_CS.S=stepinfo(DR_CS.full_tf);
step(DR_CS.full_tf);
grid on;
title("DR CS")
figure
DCM_CS.S=stepinfo(DCM_CS.full_tf);
step(DCM_CS.full_tf);
grid on;
title("DCM CS")


