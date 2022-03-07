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

% DC Motor Speed Control System Parameters
DCM.K = 0.01;
DCM.J = 0.01;
DCM.b = 0.1;
DCM.R = 1;
DCM.L = 0.5;

% sampling time(s)
DR.h  = 0.2e-3;
DCM.h = 2*DR.h;
tau = 0.3e-3;

%% State Space Model
% Dual Rotary System

DR_SC.A = [0, 0, 1, 0;
           0, 0, 0, 1;
           -DR.k / DR.J1, DR.k / DR.J1, - (DR.d + DR.b) / DR.J1,   (DR.d + DR.b) / DR.J1;
           -DR.k / DR.J2, DR.k / DR.J2,   (DR.d + DR.b) / DR.J2, - (DR.d + DR.b) / DR.J2];
DR_SC.B = [0; 
           0; 
           DR.Km / DR.J1; 
           0];
DR_SC.C = [1 1 0 0];
    
% DC Motor Speed Control System
DCM_SC.A = [-DCM.b / DCM.J, DCM.K / DCM.J;
         -DCM.K / DCM.L, DCM.R / DCM.L];
DCM_SC.B = [0;
         1 / DCM.L];
DCM_SC.C = [1 0];

%% Discrete-time model without delay

% continuous-time
DR_SC.sysc = ss(DR_SC.A, DR_SC.B, DR_SC.C, 0);
DCM_SC.sysc= ss(DCM_SC.A, DCM_SC.B, DCM_SC.C, 0);

% to discrete
DR_SC.sysd = c2d(DR_SC.sysc, DR.h);
DCM_SC.sysd = c2d(DCM_SC.sysc, DCM.h);

DR_SC.phi = DR_SC.sysd.a; DR_SC.Gamma = DR_SC.sysd.b; DR_SC.Cd = DR_SC.sysd.c;

DCM_SC.phi = DCM_SC.sysd.a; DCM_SC.Gamma = DCM_SC.sysd.b; DCM_SC.Cd = DCM_SC.sysd.c;

% Desired closed-loop poles and pole placement
DR_SC.alpha = [0.95 0.98 0.98 0.98];

DCM_SC.alpha = [0.998 0.998];

% feedback vector
DR_SC.K = -acker(DR_SC.phi, DR_SC.Gamma, DR_SC.alpha);

DCM_SC.K = -acker(DCM_SC.phi, DCM_SC.Gamma, DCM_SC.alpha);

% feedforward gain
temp = inv(eye(4) - DR_SC.phi - DR_SC.Gamma * DR_SC.K)*DR_SC.Gamma;
DR_SC.F = 1/(DR_SC.C * temp);

temp = inv(eye(2) - DCM_SC.phi - DCM_SC.Gamma * DCM_SC.K)*DCM_SC.Gamma;
DCM_SC.F = 1/(DCM_SC.C * temp);
clear temp

%% DR with delay

% DR_SC_aug.Gamma0 = inv(DR_SC.A)*(expm(DR_SC.A*(DR.h-tau))-expm(DR_SC.A*0))*DR_SC.B;
% DR_SC_aug.Gamma1 = inv(DR_SC.A)*(expm(DR_SC.A*DR.h)-expm(DR_SC.A*(DR.h-tau)))*DR_SC.B;
DR_SC_aug.Gamma0 = tau*DR_SC.B;
DR_SC_aug.Gamma1 = (DR.h - tau)*DR_SC.B;

% augmentation
DR_SC_aug.phi= [DR_SC.phi DR_SC_aug.Gamma1; zeros(1,5)];
DR_SC_aug.Gamma= [DR_SC_aug.Gamma0; 1];
DR_SC_aug.C= [DR_SC.C 0];
% Desired closed-loop poles and pole placement
DR_SC_aug.alpha = [0.95 0.98 0.98 0.98 0.9];
DR_SC_aug.K = -acker(DR_SC_aug.phi, DR_SC_aug.Gamma, DR_SC_aug.alpha);
% feedforward gain
DR_SC_aug.F = 1/(DR_SC_aug.C*inv(eye(5)-DR_SC_aug.phi-DR_SC_aug.Gamma*DR_SC_aug.K)*DR_SC_aug.Gamma);

%% DCM with delay

DCM_SC_aug.Gamma0 = inv(DCM_SC.A)*(expm(DCM_SC.A*(DCM.h-tau))-expm(DCM_SC.A*0))*DCM_SC.B;
DCM_SC_aug.Gamma1 = inv(DCM_SC.A)*(expm(DCM_SC.A*DCM.h)-expm(DCM_SC.A*(DCM.h-tau)))*DCM_SC.B;
% augmentation
DCM_SC_aug.phi= [DCM_SC.phi DCM_SC_aug.Gamma1; zeros(1,3)];
DCM_SC_aug.Gamma= [DCM_SC_aug.Gamma0; 1];
DCM_SC_aug.C= [DCM_SC.C 0];
% Desired closed-loop poles and pole placement
DCM_SC_aug.alpha = [0.998 0.997 0.996];
DCM_SC_aug.K = -acker(DCM_SC_aug.phi, DCM_SC_aug.Gamma, DCM_SC_aug.alpha);
% feedforward gain
DCM_SC_aug.F = 1/(DCM_SC_aug.C*inv(eye(3)-DCM_SC_aug.phi-DCM_SC_aug.Gamma*DCM_SC_aug.K)*DCM_SC_aug.Gamma);

%% plot
tag = 4;
switch tag
    case 1
        x0 = [0;0;0;0;];
        SC_plot(DR, DR_SC, x0, 'DR');
    case 2
        x0 = [0;0;];
        SC_plot(DCM, DCM_SC, x0, 'DCM');
    case 3
        x0 = [0;0;0;0;0;];
        SC_plot(DR, DR_SC_aug, x0, 'DR');
    case 4    
        x0 = [0;0;0;];
        SC_plot(DCM, DCM_SC_aug, x0, 'DCM');
end

clear x0