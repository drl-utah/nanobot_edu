clear; clc; close all; instrreset;
%% Connect to Device
r = MKR_MotorCarrier;

%% Connect your piezo buzzer with the red wire in M3+, black wire in M3-
% noteFrequencies = []; %Scale frequency array
%r.piezoTone(period,duration); period in MICROSECONDS, duration in MILLISECONDS