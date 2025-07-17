function [rho,acousticSpeed] = getRhoBritish(z)
%% Constants
z0 = 0; % ground altitude (MSL) where P0 and T0 are measured
T0 = 518.67; % degrees Rankine
P0 = 2116; % lb/square foot
g = 32.174; % acceleration due to gravity ft/sec^2
R = 1716.49; % specific gas constant for air ft-lb/slug-degree R
lapseRate = -.003566; % lapse rate degree-R/ft (z<~36 kft)
gammaAir = 1.4; % adiabatic constant
%% Atmosphere Model
T = T0 + lapseRate*(z-z0); % Temperature at altitude z, degree R
P = P0*(T/T0)^(-g/(lapseRate*R)); % Pressure at altitude z, lb/square ft
rho = P/(R*T); % Density at altitude z, slugs/cubic foot
acousticSpeed = sqrt(gammaAir*R*T); % Acoustic speed, ft/sec
end