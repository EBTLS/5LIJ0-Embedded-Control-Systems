%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 1
%%% Author: Jiaxuan Zhang
%%%%%

%% Parameters
% Dual Rotary System Parameters
DR.Km = 4.4 * 1e-2;
DR.J1 = 3.75 * 1e-6;
DR.J2 = 3.75 * 1e-6;
DR.b = 1 * 1e-5;
DR.k = 0.2656;
DR.d = 3.125 * 1e-5;

% DC Motor Speed Control System Parameters
DCM.K = 0.01;
DCM.J = 0.01;
DCM.b = 0.1;
DCM.R = 1;
DCM.L = 0.5;

%% State Space Model
% Dual Rotary System

DR.A = [0, 0, 1, 0;
    0, 0, 0, 1;
    -DR.k / DR.J1, DR.k / DR.J1, - (DR.d + DR.b) / DR.J1, (DR.d + DR.b) / DR.J1;
    -DR.k / DR.J2, DR.k / DR.J2, (DR.d + DR.b) / DR.J2, - (DR.d + DR.b) / DR.J2];
DR.B = [0; 0; DR.Km / DR.J1; 0];

% DC Motor Speed Control System
DCM.A = [-DCM.b / DCM.J, DCM.K / DCM.J;
    -DCM.K / DCM.L, DCM.R / DCM.L];
DCM.B = [0, 1 / DCM.L];


