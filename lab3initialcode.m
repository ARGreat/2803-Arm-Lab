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

% Extra stuff for lsim
tstart = 0;
tstep = 0.01;
tmax = 10-tstep;
time = tstart:tstep:tmax; % Time vector of simulation time
desiredtheta = 0.5; % [rad], desired radians

uchoice = 1;

if uchoice == 1
    u = desiredtheta.*ones(size(time)); % Vector of desired radians for 1 val
else
    % Alternate from 0.5 to -0.5 in half of time
    u = zeros(1,length(time));
    u(:,1:length(time)/2 - 1) = 0.5.*ones(1,length(time)/2 - 1); 
    u(:,length(time)/2:length(time)) = -0.5.*ones(1,length(time)/2 + 1);
end

for i = 1:Ns
    % Numerator and Denominator Values
    num = n1(i);
    den = [d2 d1(i) d0(i)];
    % System for input
    sysTF = tf(num,den);
    % Step sim
    [x,t] = step(sysTF);
    % Lsim sim
    x2 = lsim(sysTF,u,time);
    xdatastepfunc.(names(i)) = x./2;
    tdatastepfunc.(names(i)) = t./2;
    xdatalsimfunc.(names(i)) = x2;
end

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
figure(); hold on;
for i=1:Ns
     plot(time,xdatalsimfunc.(names(i)));
end
title('Lsim Response for Varied Proportional and Derivative Gains');
xlabel('Time (s)');
ylabel('Theta (rad)');
legend(names,location="best");

%% 2.4 Designing K1 and K3 for less than 20% overshoot, 5% settling error in < 1 sec
% Gains Values
Kpthetatest = 18; % K1 - proportional
Kdthetatest = 1.5; % K3 - derivative, max 1.5

% Values for system
n1test = Kpthetatest .* Kg * Km / (J*Rm);
d2test = 1;
d1test = (Kg^2*Km^2/(J*Rm) + Kdthetatest.*Kg*Km/(J*Rm));
d0test = Kpthetatest.*Kg*Km/(J*Rm);

% Extra stuff for lsim
tstarttest = 0;
tsteptest = 0.01;
tmaxtest = 10 - tsteptest;
timetest = tstarttest:tsteptest:tmaxtest; % Time vector of simulation time
desiredthetatest = 0.5; % [rad], desired radians

utest = zeros(1,length(timetest));
utest(:,1:length(timetest)/2 - 1) = 0.5.*ones(1,length(timetest)/2 - 1); 
utest(:,length(timetest)/2:length(timetest)) = -0.5.*ones(1,length(timetest)/2 + 1);

% Numerator and Denominator Values
numtest = n1test;
dentest = [d2test d1test d0test];

% System for Input
sysTFtest = tf(numtest,dentest);

% X values for test from lsim
xtest = lsim(sysTFtest,utest,timetest);

% Creating Graph With Bounds
figure(); hold on;
plot(timetest,xtest,'k',LineWidth=2);
title(sprintf('Lsim Response for Testing Gains (%0.2f K1, %0.2f K3)',Kpthetatest,Kdthetatest));
xlabel('Times (s)');
ylabel('Theta (rad)');

% 20% bounds for 0.5 rad and -0.5 rad
yline(desiredthetatest*0.2 + desiredthetatest,'--r');
yline(-desiredthetatest*0.2 + desiredthetatest,'--r');
yline(desiredthetatest*0.2 - desiredthetatest,'--r');
yline(-desiredthetatest*0.2 - desiredthetatest,'--r');

% 5% bounds for 0.5 rad and -0.5 rad
yline(desiredthetatest*0.05 + desiredthetatest,'--g');
yline(-desiredthetatest*0.05 + desiredthetatest,'--g');
yline(desiredthetatest*0.05 - desiredthetatest,'--g');
yline(-desiredthetatest*0.05 - desiredthetatest,'--g');

% Step input time (0 and 10 sec)
xline(0,'--m');
xline(5,'--m');

% 1 sec settling lines (1 and 11 sec)
xline(1,'--b');
xline(6,'--b');

% Legend
legend("Theoretical Data","20% Bounds","","","","5% Settling Bounds","","","","Input","","1 Second to Settle","",location="eastoutside");

% Bounds for Graphs
xlim([-0.1 10])
ylim([-0.7 0.7]);
