%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 1 Design 1
%%%%%

clc;
clear all;
format short;

close all;


%% configuration & time analysis
% setup
% frame rate 100 frames per second, fh/s
LKAS.fh = 1/100;
LKAS.n_ROI = 8;
LKAS.contorller_type = 1;

% case selection
Case_index = 5;

% plot setting 
PATTERN = {1};

Reference = 0;
% simulation in seconds
Simulation_time = 10; 
Initial_value = 0.15;

switch Case_index
    case 1
        disp('Case 1:');
        LKAS.n_pipeline = 1;
        LKAS.n_parallelization = 1;
    
    case 2
        disp('Case 2:')
        LKAS.n_pipeline = 1;
        LKAS.n_parallelization = 5;
    
    case 3
        disp('Case 3:')
        LKAS.n_pipeline = 8;
        LKAS.n_parallelization = 1;
    
    case 4
        disp('Case 4:')
        LKAS.n_pipeline = 2;
        LKAS.n_parallelization = 1;
    
    case 5
        disp('Case 5:')
        LKAS.n_pipeline = 2;
        LKAS.n_parallelization = 2;
        PATTERN={'s_1'};
        
    otherwise
        disp('invalid index, set as default case 1')
        LKAS.n_pipeline = 1;
        LKAS.n_parallelization = 1;
end


% time analysis 
LKAS = timeAnalysis(LKAS);
fprintf('System time analysis: tau=%.3f, h=%.3f\n',LKAS.tau, LKAS.h);


%% controller design
[LKAS_CS.A, LKAS_CS.B, LKAS_CS.C, LKAS_CS.D, LKAS_CS.nx] = systemModel();

% agumentation
if (LKAS.n_pipeline <= 1)
    LKAS_CS = augmentSystem(LKAS.h, LKAS.h, LKAS.n_pipeline, LKAS_CS);
else
    LKAS_CS = augmentSystem(LKAS.tau, LKAS.h, LKAS.n_pipeline, LKAS_CS);
end
LKAS_CS.R = 1;

temp = [0.76 0.22 0.61 0.015 0 0 0 0 ];
temp = [0 0.23 0.23 0 0];


LKAS_CS.Q = 1*(temp') * temp;
clear temp
[LKAS_CS.K, LKAS_CS.F, LKAS_CS.K_T, LKAS_CS.T] = discreteTimeController(LKAS.tau, LKAS.h, LKAS_CS.Q, LKAS_CS.R, LKAS_CS);


%% Simulation

if LKAS.n_pipeline <= 1
    simulateDIC(Initial_value, LKAS.contorller_type, LKAS.h, LKAS.tau, LKAS_CS.Phi_aug, LKAS_CS.Gamma_aug, LKAS_CS.C_aug, LKAS_CS.K, LKAS_CS.F, PATTERN, Simulation_time, Reference, LKAS.fh);
else
    simulatePipelinedDIC(Initial_value, LKAS.contorller_type, LKAS.h, LKAS.tau, LKAS_CS.Phi_aug, LKAS_CS.Gamma_aug, LKAS_CS.C_aug, LKAS_CS.K, LKAS_CS.F, Simulation_time, Reference)
end

clear PATTERN



