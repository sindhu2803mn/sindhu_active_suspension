# sindhu_active_suspension
clc;
clear;
close all;

%% =========================================================
%  ADAPTIVE ACTIVE VIBRATION STABILIZATION SYSTEM
%  Objective:
%  1. Automate disturbances
%  2. Achieve settling time < 5 seconds
%  3. Compare open-loop and closed-loop performance
%% =========================================================

%% PLANT MODEL
% G(s) = 1 / (s^2 + 3s + 2)

num = [1];
den = [1 3 2];

plant = tf(num,den);

disp('PLANT TRANSFER FUNCTION:')
plant

%% =========================================================
%% PD CONTROLLER DESIGN

Kp = 8;
Kd = 3;

controller = pid(Kp,0,Kd);

disp('PD CONTROLLER:')
controller

%% =========================================================
%% CLOSED LOOP SYSTEM

closed_sys = feedback(controller*plant,1);

disp('CLOSED LOOP SYSTEM:')
closed_sys

%% =========================================================
%% AUTOMATED DISTURBANCE PROFILE

% Simulation time
t = 0:0.01:15;

% Initialize disturbance signal
u = zeros(size(t));

% Disturbance 1 : Road bump
u(t>=1) = u(t>=1) + 1;

% Disturbance 2 : Uneven terrain
u(t>=4) = u(t>=4) + 0.5;

% Disturbance 3 : Sudden shock event
u(t>=7) = u(t>=7) + 2;

% Disturbance 4 : Recovery phase
u(t>=10) = u(t>=10) - 1.5;

%% =========================================================
%% SYSTEM RESPONSES

% Closed-loop response
[y_closed,t_closed] = lsim(closed_sys,u,t);

% Open-loop response
[y_open,t_open] = lsim(plant,u,t);

%% =========================================================
%% DISTURBANCE SIGNAL PLOT

figure;
plot(t,u,'LineWidth',2);
grid on;

title('Automated Disturbance Profile');
xlabel('Time (s)');
ylabel('Disturbance Amplitude');

%% =========================================================
%% CLOSED LOOP RESPONSE

figure;
plot(t_closed,y_closed,'LineWidth',2);
grid on;

title('Closed Loop Response with PD Controller');
xlabel('Time (s)');
ylabel('System Output');

%% =========================================================
%% OPEN LOOP RESPONSE

figure;
plot(t_open,y_open,'LineWidth',2);
grid on;

title('Open Loop Response without Controller');
xlabel('Time (s)');
ylabel('System Output');

%% =========================================================
%% COMPARISON PLOT

figure;
plot(t_open,y_open,'LineWidth',2);
hold on;
plot(t_closed,y_closed,'LineWidth',2);

grid on;

title('Open Loop vs Closed Loop Comparison');
xlabel('Time (s)');
ylabel('System Response');

legend('Open Loop','Closed Loop');

%% =========================================================
%% PERFORMANCE METRICS

disp('===========================================')
disp('CLOSED LOOP PERFORMANCE METRICS')
disp('===========================================')

info = stepinfo(closed_sys)

%% =========================================================
%% SETTLING TIME CHECK

if info.SettlingTime < 5
    disp('SUCCESS: Settling time requirement achieved!')
else
    disp('Settling time exceeds 5 seconds.')
end

%% =========================================================
%% ROOT LOCUS ANALYSIS

figure;
rlocus(controller*plant);
grid on;

title('Root Locus of Controlled System');

%% =========================================================
%% BODE PLOT

figure;
bode(closed_sys);
grid on;

title('Bode Plot of Closed Loop System');

%% =========================================================
%% POLE-ZERO MAP

figure;
pzmap(closed_sys);
grid on;

title('Pole-Zero Map');

%% =========================================================
%% POLE DISPLAY

disp('===========================================')
disp('CLOSED LOOP POLES')
disp('===========================================')

pole(closed_sys)

%% =========================================================
%% FINAL ENGINEERING SUMMARY

disp('===========================================')
disp('ENGINEERING SUMMARY')
disp('===========================================')

fprintf('Kp = %.2f\n',Kp);
fprintf('Kd = %.2f\n',Kd);
fprintf('Settling Time = %.2f seconds\n',info.SettlingTime);
fprintf('Overshoot = %.2f %%\n',info.Overshoot);
fprintf('Rise Time = %.2f seconds\n',info.RiseTime);

disp('System successfully rejects automated disturbances using PD control.')
