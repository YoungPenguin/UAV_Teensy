%% parameters for control experiments for flying propeller
% sample time
Ts = 0.002; % måling interval (sampletime) - sek
% motorværdier
% EMAX GT2210/09 
mRPMv = 1780; % RPM pr volt
Km = 60/(mRPMv * 2 * pi); % motorkonstant [V/(rad/s)] eller [Nm/A]
% trust med propel 8x4
% ud fra 600g trust ved 11200 RPM (13A), 2s LiPo battei (8V)
backEMF = 11200/mRPMv;
RaVolt = 8.2 - backEMF; % spænding over ankermodstand
Ra = RaVolt/11; % ankermodstand ved 11A
% trust pr rad/s (8x4 propel) 0.6 kg ved 11200 RPM
Ktrust = (0.6 * 9.8)/(11200 / 60 * 2 * pi); % trust i Newton
% drag ved 11200 RPM
KDrag = 11 * Km / (11200 / 60 * 2 * pi);
% done





