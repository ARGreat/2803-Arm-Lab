%% ASEN 2803: Rotary Control Lab
% Section 004, Group 09
clear all; close all; clc;

%% Input Variables
% Base Variables
Kg = 33.3; % Gear Ratio
Km = 0.0401; % [V/(rad/sec) or Nm/amp] Motor Constant
Rm = 19.2; % [ohms], Armature Resistance
Ns = 6; % Number of sets

% Rigid Arm Variables
J_hub = 0.0005; % [Kgm^2], Base inertia
J_extra = (0.2*0.2794^2); % [Kgm^2]
J_load = 0.0015; % [Kgm^2], load inertia of the bar
J = J_hub + J_extra + J_load; % [Kgm^2], total inertia

% Flexible Link Variables
L = 0.45; % [m], link length
Marm = 0.06; % [kg], link mass of ruler
J_arm = (Marm*L^2)/3; % [0.004, Kgm^2], link rigid body inertia
Mtip = 0.05; % [Kg], tip mass
J_M = Mtip*L^2; % [0.01, Kgm^2], tip mass inertia
fc = 1.8; % [Hz], natural frequency
J_L = J_arm + J_M; % Inertia of Flex Link Mass
Karm = (2*pi*fc)^2*(J_L); % Flexible link stiffness

% Adjustable Variables
Kptheta = [10 20 5 10 10 10]; % K1-proportional
Kdtheta = [0 0 0 1 -1 -0.5]; % K3-derivative

%% Calculations for n1, d2, d1, d0
n1 = Kptheta .* Kg * Km / (J*Rm);
d2 = 1;
d1 = (Kg^2*Km^2/(J*Rm) + Kdtheta.*Kg*Km/(J*Rm));
d0 = Kptheta.*Kg*Km/(J*Rm);

%% Closed Loop System
% Step function values, multiplied by 0.5 to get to 0.5 amplitude
names = ["Set1", "Set2", "Set3", "Set4", "Set5", "Set6"];

for i = 1:Ns
    num = n1(i);
    den = [d2 d1(i) d0(i)];
    sysTF = tf(num,den);
    [x,t] = step(sysTF);
    xdatastepfunc.(names(i)) = x./2;
    tdatastepfunc.(names(i)) = t./2;
end

% Lsim values

%% Graphing
% Graph for Step function values
figure(); hold on;
for i=1:Ns
    plot(tdatastepfunc.(names(i)),xdatastepfunc.(names(i)));
end
title('Step Response for Varied Proportional and Derivative Gains');
xlabel('Time (s)');
ylabel('Theta (rad)');
legend(names);

% Graph for lsim