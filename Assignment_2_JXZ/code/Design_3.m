%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 2 Design 2
%%% Author: Jiaxuan Zhang
%%%%%

%% Env Configuration
addpath('./src')

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
fh = 1/100;

%% System Model
[Case_5.A, Case_5.B, Case_5.C, Case_5.D] = systemModel();
Case_5

%% Case 5: Mixed
Case_5.tau = (isp1 + isp2 * z2/4 + isp3 + RoID + RoIP * z_ip/4 + RoIM + Controller + Actuator)/1e3; % 36.4
Case_5.h = ceil(Case_5.tau / (2 * fh)) * fh; % 20,

% Controller Design
case_id=5;
[Case_5.phi_aug, Case_5.Gamma_aug, Case_5.C_aug, Case_5.K, Case_5.F, Case_5.K_T, Case_5.T] = discreteTimeController(Case_5.tau, Case_5.h,case_id);
Case_5

% simulation configuration
CONTROLLER_TYPE = 1;
SIMULATION_TIME = 10;
Reference= 0;
PATTERN = {1};

Case_5.phi_aug={Case_5.phi_aug};
Case_5.Gamma_aug={Case_5.Gamma_aug};
Case_5.C_aug={Case_5.C_aug};
Case_5.K={Case_5.K};
Case_5.F={Case_5.F};
Case_5.K_T={Case_5.K_T};
Case_5.T={Case_5.T};

simulatePipelinedDIC(Case_5.h,Case_5.tau,Case_5.phi_aug,Case_5.Gamma_aug,Case_5.C_aug,Case_5.K,Case_5.F,SIMULATION_TIME,Reference,CONTROLLER_TYPE);

