%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 1: Timing Analysis
%%% Author: Jiaxuan Zhang
%%%%%

%% Parameter
Core_Frequency=40*1e6;

%% DR CS

% with delay
% WCET ms
% h=0.002, tau=4e-4
DR_WCET_1=37075/Core_Frequency*1e3

% h=0.002, tau=1e-3
DR_WCET_2=36175/Core_Frequency*1e3

% h=0.025, tau=4e-4
DR_WCET_3=37200/Core_Frequency*1e3

%% DCM CS

% with delay
% WCET ms
% h=0.002 tau=10e-4
DCM_WCET_2_10 = 31150/Core_Frequency*1e3;
% 31150 31150 31150

% h=0.002 tau=4e-4
DCM_WCET_2_4 = 31200/Core_Frequency*1e3;
% 31250 31150 31250 31150

% h=0.002 tau=2e-4
DCM_WCET_2_2 = 31150/Core_Frequency*1e3;
% 31150 31150 31150

% h=0.002 tau=0
DCM_WCET_2_0 = 31150/Core_Frequency*1e3;
% 31150 31150 31150

% h=0.001 
% tau=8e-4
DCM_WCET_1_10 = 31150/Core_Frequency*1e3;
% 31225 31150 31150

% tau=2e-4
DCM_WCET_1_2 = 31150/Core_Frequency*1e3;
% 31150

% tau=0
DCM_WCET_1_0 = 31150/Core_Frequency*1e3;
% 31150 31150 31150

% h=0.002 tau=1e-3
DCM_WCET_2=31150/Core_Frequency*1e3;

% h=0.025, tau=4e-4
DCM_WCET_3=32250/Core_Frequency*1e3;


%% TDMA Timing
slot_1=2*1e4;
slot_2=56*1e3;
slot_3=71*1e3;

part_1=slot_1/Core_Frequency*1e3
part_2=slot_2/Core_Frequency*1e3
part_3=slot_3/Core_Frequency*1e3

h=(slot_1+slot_2+slot_3)/Core_Frequency*1e3