%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 1: Timing Analysis
%%% Author: Jiaxuan Zhang
%%%%%

%% Parameter
Core_Frequency = 40 * 1e6;

%% DR CS
DR_WCET = [];

% h=3, tau=2
temp_WCET = 36950 / Core_Frequency * 1e3;
DR_WCET = [DR_WCET, temp_WCET];

% h=3, tau=1
temp_WCET = 37100 / Core_Frequency * 1e3;
DR_WCET = [DR_WCET, temp_WCET];

% h=3, tau=0.5
% unstable

% h=3, tau=0
% unstable

% h=2, tau=1
temp_WCET = 36175 / Core_Frequency * 1e3;
DR_WCET = [DR_WCET, temp_WCET];

% h=2, tau=0.5
temp_WCET = 36400 / Core_Frequency * 1e3;
DR_WCET = [DR_WCET, temp_WCET];

% h=2, tau=0.4
temp_WCET = 37075 / Core_Frequency * 1e3;
DR_WCET = [DR_WCET, temp_WCET];

% h=2, tau=0
temp_WCET = 37150 / Core_Frequency * 1e3;
DR_WCET = [DR_WCET, temp_WCET];

% h=2.5, tau=0.4
temp_WCET = 37200 / Core_Frequency * 1e3;
DR_WCET = [DR_WCET, temp_WCET];

% h=1, tau=0.5
temp_WCET = 35300 / Core_Frequency * 1e3;
DR_WCET = [DR_WCET, temp_WCET];

% h=1, tau=0.3

% h=1, tau=0

DR_WCET

%% DCM CS
% with delay
DCM_WCET = [];

% h=3 tau=2
temp_WCET = 32250 / Core_Frequency * 1e3;
DCM_WCET = [DCM_WCET, temp_WCET]

% h=0.002 tau=10e-4
temp_WCET = 31150 / Core_Frequency * 1e3;
DCM_WCET = [DCM_WCET, temp_WCET]
% 31150 31150 31150

% h=0.002 tau=4e-4
DCM_WCET_2_4 = 31200 / Core_Frequency * 1e3;
% 31250 31150 31250 31150

% h=0.002 tau=2e-4
DCM_WCET_2_2 = 31150 / Core_Frequency * 1e3;
% 31150 31150 31150

% h=0.002 tau=0
DCM_WCET_2_0 = 31150 / Core_Frequency * 1e3;
% 31150 31150 31150

% h=0.001
% tau=8e-4
DCM_WCET_1_10 = 31150 / Core_Frequency * 1e3;
% 31225 31150 31150

% tau=2e-4
DCM_WCET_1_2 = 31150 / Core_Frequency * 1e3;
% 31150

% tau=0
DCM_WCET_1_0 = 31150 / Core_Frequency * 1e3;
% 31150 31150 31150

% h=0.002 tau=1e-3
DCM_WCET_2 = 31150 / Core_Frequency * 1e3;

%% TDMA Timing
slot_1 = 2 * 1e4;
slot_2 = 56 * 1e3;
slot_3 = 71 * 1e3;

part_1 = slot_1 / Core_Frequency * 1e3;
part_2 = slot_2 / Core_Frequency * 1e3;
part_3 = slot_3 / Core_Frequency * 1e3;

h = (slot_1 + slot_2 + slot_3) / Core_Frequency * 1e3;

%% TIME Computing
DCM_T = 32000 / Core_Frequency * 1e3
DR_T = 37500 / Core_Frequency * 1e3
Context_Switch = 2000 / Core_Frequency * 1e3
Sys_App_T = 5000 / Core_Frequency * 1e3

%% Design 1 TDM1
Design_1_T1.p = []
Design_1_T1.p1 = 37500;
Design_1_T1.p2 = 32000;
Design_1_T1.p3 = 80000 - Design_1_T1.p1 - Design_1_T1.p2 - (2 + 1) * 2000;
Design_1_T1.p4 = 37500;
Design_1_T1.p5 = 80000 - Design_1_T1.p4 - 2000 - 2000 - 5000 - 2000;
Design_1_T1.p = [Design_1_T1.p1, Design_1_T1.p2, Design_1_T1.p3, Design_1_T1.p4, Design_1_T1.p5]
Design_1_T1.h1 = (Design_1_T1.p1 + Design_1_T1.p2 + Design_1_T1.p3 + Design_1_T1.p4 + 4 * 2000 + 5000) / Core_Frequency * 1e3;
Design_1_T1.tau1 = Design_1_T1.p1 / Core_Frequency * 1e3;
Design_1_T1.h2 = 2 * Design_1_T1.h1;
Design_1_T1.tau2 = (Design_1_T1.p2 + Design_1_T1.p3 + Design_1_T1.p4 + 3 * 2000) / Core_Frequency * 1e3;

%% Design
Design_1_T2.p = [];
Design_1_T2.p1 = 37500;
Design_1_T2.p2 = 16000;
Design_1_T2.p3 = 37500;
Design_1_T2.p4 = 16000;
Design_1_T2.p = [Design_1_T2.p1, Design_1_T2.p2, Design_1_T2.p3, Design_1_T2.p4];
Design_1_T2.h1 = (Design_1_T2.p1 + Design_1_T2.p2 + 5000 + 3 * 2000) / Core_Frequency * 1e3;
Design_1_T2.tau1 = Design_1_T2.p1 / Core_Frequency * 1e3;
Design_1_T2.h2 = 2 * Design_1_T2.h1;
Design_1_T2.tau2 = (Design_1_T2.p2 + Design_1_T2.p3 + Design_1_T2.p4 + 3 * 2000) / Core_Frequency * 1e3
