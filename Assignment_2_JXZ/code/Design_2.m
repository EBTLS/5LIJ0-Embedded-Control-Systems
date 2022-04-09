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
[Case_3.A, Case_3.B, Case_3.C, Case_3.D] = systemModel();
Case_3
[Case_4.A, Case_4.B, Case_4.C, Case_4.D] = systemModel();
Case_4

%% Case 3: Pipeline
Case_3.tau = (isp1 + isp2 * z2 + isp3 + RoID + RoIP * z_ip + RoIM + Controller + Actuator)/1e3; % 96.6760ms
Case_3.h = ceil(Case_3.tau / (8 * fh)) * fh; % 20, which means 5 pipelines is already enough
Case_3.tau=100*1e-3;

% Controller Design
case_id=3;
[Case_3.phi_aug, Case_3.Gamma_aug, Case_3.C_aug, Case_3.K, Case_3.F, Case_3.K_T, Case_3.T] = discreteTimeController(Case_3.tau, Case_3.h,case_id);
Case_3

% simulation configuration
CONTROLLER_TYPE = 1;
SIMULATION_TIME = 10;
Reference= 0;
PATTERN = {1};

Case_3.phi_aug={Case_3.phi_aug};
Case_3.Gamma_aug={Case_3.Gamma_aug};
Case_3.C_aug={Case_3.C_aug};
Case_3.K={Case_3.K};
Case_3.F={Case_3.F};
Case_3.K_T={Case_3.K_T};
Case_3.T={Case_3.T};

simulatePipelinedDIC(Case_3.h,Case_3.tau,Case_3.phi_aug,Case_3.Gamma_aug,Case_3.C_aug,Case_3.K,Case_3.F,SIMULATION_TIME,Reference,CONTROLLER_TYPE);
%% Case 4
case_id=4;
for core_number=5
    
    Case_4.tau = (isp1 + isp2 * z2 + isp3 + RoID + RoIP * z_ip + RoIM + Controller + Actuator)/1e3; % 96.6760
    Case_4.h = ceil(Case_4.tau / (core_number * fh)) * fh; 

    % Controller Design
    [Case_4.phi_aug, Case_4.Gamma_aug, Case_4.C_aug, Case_4.K, Case_4.F, Case_4.K_T, Case_4.T] = discreteTimeController(Case_4.tau, Case_4.h,case_id);
    Case_4

    % simulation configuration
    CONTROLLER_TYPE = 1;
    SIMULATION_TIME = 10;
    Reference= 0;
    PATTERN = {1};

    Case_4.phi_aug={Case_4.phi_aug};
    Case_4.Gamma_aug={Case_4.Gamma_aug};
    Case_4.C_aug={Case_4.C_aug};
    Case_4.K={Case_4.K};
    Case_4.F={Case_4.F};
    Case_4.K_T={Case_4.K_T};
    Case_4.T={Case_4.T};

    figure 
    simulatePipelinedDIC(Case_4.h,Case_4.tau,Case_4.phi_aug,Case_4.Gamma_aug,Case_4.C_aug,Case_4.K,Case_4.F,SIMULATION_TIME,Reference,CONTROLLER_TYPE);
    
end

