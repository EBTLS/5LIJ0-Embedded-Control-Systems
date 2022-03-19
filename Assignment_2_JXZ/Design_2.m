%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 2 Design 2
%%% Author: Jiaxuan Zhang
%%%%%

%% System Parameters
isp1 = 10.3;
isp2 = 8.65;
z2 = 8;
isp3 = 5;
RoID = 0.3;
RoIP = 1.4;
z_ip = 8;
RoIM = 0.16;
Controller = 0.016;
Actuator = 0.5

framerate = 100;
h_original = 1/100 * 1e3;

%% System Model
[Case_3.A, Case_3.B, Case_3.C, Case_3.D] = systemModel();
Case_3
[Case_4.A, Case_4.B, Case_4.C, Case_4.D] = systemModel();
Case_4

%% Case 3: Pipeline
Case_3.tau = isp1 + isp2 * z2 + isp3 + RoID + RoIP * z_ip + RoIM + Controller + Actuator; % 96.6760
Case_3.h = ceil(Case_3.tau / (8 * h_original)) * h_original; % 20, which means 5 pipelines is already enough

% [Case_3.phi_aug,Case_3.Gamma_aug,Case_3.C_aug]=augmentSystem(Case_3.tau,Case_3.h,1)

[Case_3.phi_aug,Case_3.Gamma_aug,Case_3.C_aug,Case_3.K,Case_3.F,Case_3.K_T,Case_3.T] = discreteTimeController(Case_3.tau,Case_3.h)

%% Case 2
