%% parameters for control experiments
close all
clear
%
%% motorværdier
% EMAX GT2210/09 with 9x4 propeller
Kv = 1780; % RPM pr volt
Km = 60/(Kv * 2 * pi); % motor constant [V/(rad/s)] or [Nm/A]
% Motor/propeller data from supplier
batVolt = 8.0; % assumed battery voltage for 2S battery
rpmMax = 3508; % max RPM - assumed full battery voltage over motor
ampMax = 5; % current at max RPM in amps
trustMax = 1.6; % trust in kg
% 
g = 9.80665; % m/s^2 gravity acceleration
backEMF = rpmMax/Kv; % voltage generated from rotation
RaVolt = batVolt - backEMF; % remainig voltage over resistance
Ra = RaVolt/ampMax; % (winding) resistance [Ohm]
% effect of propeller in trust and drag
Ktrust = (trustMax * g)/(rpmMax / 60 * 2 * pi); % trust i N/(rad/s)
% drag at max 
KDrag = ampMax * Km / (rpmMax / 60 * 2 * pi); % drag per rad/sec [Nms]
%% hover 
heightRef = 1.0;
% sample time
Ts = 0.002; % måling interval (sampletime) - sek
%% drone konstants
bodyHalf = 0.3; % half drone weight (no motors) [kg]
motorMass = 0.055; % mass of one motor [kg]
propellerMass = 0.015; % scaled up to get more realistic inertia
trustPerPropeller = (bodyHalf/2 + motorMass + propellerMass) * g; % [N]
% hover calculation
hoverVel = trustPerPropeller / Ktrust; % in radians/sec
hoverRPM = hoverVel/(2*pi) * 60; % converted to RPM 
% hover drag
hoverDrag = KDrag * hoverVel; % [Nm]
hoverCurrent = hoverDrag/Km; % [A]
hoverVoltage = hoverCurrent * Ra + hoverRPM / Kv;