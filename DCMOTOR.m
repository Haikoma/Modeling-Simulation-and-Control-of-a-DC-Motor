%%
clearvars; close all; clc
%%
% Simulation for Input voltage of 10
t=linspace(0,10,100);
[t1,Vc1] = ode45(@(t,x) dcmotor(t,x),t,[0 0]);
figure(2); plot(t1,Vc1(:,1)*60/2/pi); xlabel('Time, s', 'Interpreter','latex'); ylabel('$rotations$','Interpreter','latex'); grid on;
legend('$\omega$', 'interpreter', 'latex','fontsize',12)

s = tf('s');
J = ureal('J', 0.01,  'Percentage',10);
b = 0.1;
K = 0.01;
R = ureal('R', 1,  'Percentage',10);
L = 0.5;

% State space and transfer function representation
A = [-b/J   K/J
    -K/L   -R/L];
B = [0
    1/L];
C = [1   1];
D = 0;
DC_Motor = ss(A,B,C,D);
nom_DC_Motor=DC_Motor.NominalValue;

%bode(DC_Motor); 
%pzplot(DC_Motor);
%step(DC_Motor);

P_motor = K/((J.NominalValue*s+b)*(L*s+R.NominalValue)+K^2);
% [C, info] = pidtune(P_motor, 'pid')
%% PID controller
% REQS : for a step input response of 1rad/s
% Settling time less than 2 seconds
% Overshoot less than 5%
% Steady-state error less than 1%

% Design PID Controller
desired_overshoot = 5;  % Overshoot limit in percentage
desired_ss_error = 2;   % Steady state error limit in percentage
desired_settling_time = 2;  % Settling time limit in seconds
% These parameters are equivalent to a second order system with:
zeta = -log(desired_overshoot/100)/sqrt(pi^2 + (log(desired_overshoot/100))^2);
wn = 4/(zeta*desired_settling_time);
F = wn^2/(s^2 + 2*s*zeta*wn + wn^2);

% Calculate PID parameters using the desired performance criteria
Kp = (2*zeta*wn)/0.06;
Ki = wn^2/0.1001;
Kd = 2;
% Create PID Controller
pid_controller = pid(Kp, Ki, Kd);
% Connect the plant and controller
sys_cl = feedback(nom_DC_Motor*pid_controller, 1);
ystepinfo = stepinfo(sys_cl);

figure;
step(sys_cl); hold on; step(F); hold on; step(nom_DC_Motor);
title('Closed-loop Step Response');
xlabel('Time (s)');
ylabel('Angular Position (rad)');
legend("Tuned respnse", "Requirement", "Nominal")
grid on;

%% Linear Quadratic control
R1 = 230;
Q1 = 230*C'*C;
Q1(2,2)=0;
[K1,S1,P1] = lqr(A.NominalValue,B,Q1,R1);

sys1 = ss(A.NominalValue-B*K1,B,C,D);
step(sys1); 
% hold on; step(nom_DC_Motor);
hold on; step(F); grid on
legend("Tuned respnse", "Requirement");

%% Robust control through mu-synthesis
C_tunable = tunablePID('C_tunable', 'PID');
C_tunable.Kp.Value=67;
C_tunable.Ki.Value=83;
C_tunable.Kd.Value=3;
sys_cl_unc = feedback(C_tunable*DC_Motor,1);
[CL,CLperf] = musyn(sys_cl_unc);
step(CL);axis([0 5 0 1.5])
%%
function dxdt=dcmotor(t,x) % Function to be fed to the solver
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;
r = x(1);
v = x(2);
expression2 = (R*b+K^2)/(L*J);
expression1 = (L*b+R*J)/(L*J);
expression3=(K/(L*J));
inputvoltage=1e1;
% Output
dxdt_1 = v;
dxdt_2 = -v*expression1 - r*expression2 + expression3*inputvoltage;
dxdt=[dxdt_1; dxdt_2];
end