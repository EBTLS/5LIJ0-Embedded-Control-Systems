%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 3 Case 1
%%% Author: Yiting Li, Jiaxuan Zhang
%%%%%

close all
clear all
clc

%% load resources
addpath('./src')

%% Gloabl Parameter
case_id = 1; % 1 for CAN, 2 for FlexRay
EMB.fh = 1/60;

%simulation
Initial_value = 0;
Simulation_time = 20;
PATTERN = {1};
Reference = 0.002;

%% State Space Model
% Emergency Braking System
EMB_CS.nx = 5;

EMB_CS.A = [-520,   -220,   0,          0,          0;
            220,    -500,   -999994,    0,          2e8;
            0,      1,      0,          0,          0;
            0,      0,      66667,      -0.1667,    -1.3333e7;
            0,      0,      0,          1,          0];
       
EMB_CS.B = [1000;
           0;
           0;
           0;
           0];
       
EMB_CS.C = [0 0 0 0 1];

EMB_CS.D = 0;

% sampling time(s) & delay
EMB.n_pipeline = 1;
EMB.tau = 133.33e-3;
EMB.h = 133.33e-3;
EMB.contorller_type = 1;



% agumentation
if (EMB.n_pipeline <= 1)
    EMB_CS = augmentSystem(EMB.h, EMB.h, EMB.n_pipeline, EMB_CS);
else
    EMB_CS = augmentSystem(EMB.tau, EMB.h, EMB.n_pipeline, EMB_CS);
end
EMB_CS.R = 1;

EMB_CS.Q = 3 * eye(size(EMB_CS.C_aug{1}, 2), size(EMB_CS.C_aug{1}, 2));
EMB_CS.Q(5,5) = 1e10;
[EMB_CS.K, EMB_CS.F, EMB_CS.K_T, EMB_CS.T] = discreteTimeController(EMB.tau, EMB.h, EMB_CS.Q, EMB_CS.R, EMB_CS);


if EMB.n_pipeline <= 1
    simulateDIC(Initial_value, EMB.contorller_type, EMB.h, EMB.tau, EMB_CS.Phi_aug, EMB_CS.Gamma_aug, EMB_CS.C_aug, EMB_CS.K, EMB_CS.F, PATTERN, Simulation_time, Reference, EMB.fh);
else
    simulatePipelinedDIC(Initial_value, EMB.contorller_type, EMB.h, EMB.tau, EMB_CS.Phi_aug, EMB_CS.Gamma_aug, EMB_CS.C_aug, EMB_CS.K, EMB_CS.F, Simulation_time, Reference)
end

clear PATTERN
