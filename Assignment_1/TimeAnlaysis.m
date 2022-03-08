%%%%%
%%% 5LIJ0 Embedded Control Systems
%%% Assignment 1: Timing Analysis
%%% Author: Jiaxuan Zhang
%%%%%

%% Parameter
Core_Frequency=40*1e6;

%% DR CS
DR_WCET=27375/1e9*1e3




%% DCM CS


%% TDMA Timing
slot_1=2*1e4;
slot_2=56*1e3;
slot_3=71*1e3;

h=(slot_1+slot_2+slot_3)/Core_Frequency*1e3