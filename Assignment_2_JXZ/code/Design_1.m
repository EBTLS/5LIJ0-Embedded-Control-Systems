%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 2 Design 1
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
[Case_1.A, Case_1.B, Case_1.C, Case_1.D] = systemModel();
Case_1
[Case_2.A, Case_2.B, Case_2.C, Case_2.D] = systemModel();
Case_2

%% Case 1
Case_1.tau = (isp1 + isp2 * z2 + isp3 + RoID + RoIP * z_ip + RoIM + Controller + Actuator)/1e3;
Case_1.h = (ceil(Case_1.tau / fh) * fh);

% Controller Design
case_id=1;
[Case_1.phi_aug, Case_1.Gamma_aug, Case_1.C_aug, Case_1.K, Case_1.F, Case_1.K_T, Case_1.T] = discreteTimeController(Case_1.h, Case_1.h,case_id);
Case_1

% simulation configuration
CONTROLLER_TYPE = 1;
SIMULATION_TIME = 10;
Reference= 0;
PATTERN = {1};

Case_1.phi_aug={Case_1.phi_aug};
Case_1.Gamma_aug={Case_1.Gamma_aug};
Case_1.C_aug={Case_1.C_aug};
Case_1.K={Case_1.K};
Case_1.F={Case_1.F};
Case_1.K_T={Case_1.K_T};
Case_1.T={Case_1.T};

simulateDIC(CONTROLLER_TYPE, Case_1.h, Case_1.tau, Case_1.phi_aug, Case_1.Gamma_aug, Case_1.C_aug, Case_1.K, Case_1.F, PATTERN, SIMULATION_TIME, Reference, fh);

%% Case 2: with parallelization
Case_2.Core_Number = 8;
Case_2.tau = (isp1 + isp2 * z2 / Case_2.Core_Number + isp3 + RoID + RoIP * z_ip / Case_2.Core_Number + RoIM + Controller + Actuator)/1e3;
Case_2.h = ceil(Case_2.tau / fh) * fh;

% Controller Design
case_id=2;
[Case_2.phi_aug, Case_2.Gamma_aug, Case_2.C_aug, Case_2.K, Case_2.F, Case_2.K_T, Case_2.T] = discreteTimeController(Case_2.h, Case_2.h,case_id);
Case_2

% simulation configuration
CONTROLLER_TYPE = 1;
SIMULATION_TIME = 10;
Reference= 0;
PATTERN = {1};

Case_2.phi_aug={Case_2.phi_aug};
Case_2.Gamma_aug={Case_2.Gamma_aug};
Case_2.C_aug={Case_2.C_aug};
Case_2.K={Case_2.K};
Case_2.F={Case_2.F};
Case_2.K_T={Case_2.K_T};
Case_2.T={Case_2.T};

simulateDIC(CONTROLLER_TYPE, Case_2.h, Case_2.tau, Case_2.phi_aug, Case_2.Gamma_aug, Case_2.C_aug, Case_2.K, Case_2.F, PATTERN, SIMULATION_TIME, Reference, fh);


%% Discussion
[Case_dis.A, Case_.B, Case_2.C, Case_2.D] = systemModel();
Case_2
