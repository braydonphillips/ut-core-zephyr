clear, clc, close all
T = readtable('180z.csv');
int_lsm0 = rad2deg(cumtrapz(T.t, T.lsm0_z));
int_lsm1 = rad2deg(cumtrapz(T.t, T.lsm1_z));
int_i3g0 = rad2deg(cumtrapz(T.t, T.i3g0_z));
int_i3g1 = rad2deg(cumtrapz(T.t, T.i3g1_z));
fprintf('LSM0: %6.1f°\nLSM1: %6.1f°\nI3G0: %6.1f°\nI3G1: %6.1f°\n', ...
        int_lsm0(end), int_lsm1(end), int_i3g0(end), int_i3g1(end));




% FOR X
%LSM0: -349.9°
%LSM1: -311.5°
%I3G0: -177.1°
%I3G1: -164.9°

% FOR Y
%LSM0:  371.3°
%LSM1:  398.0°
%I3G0:  179.2°
%I3G1:  174.7°

% FOR Z 
%LSM0:  353.9°
%LSM1:  405.8°
%I3G0:  171.4°
%I3G1:  182.5°




