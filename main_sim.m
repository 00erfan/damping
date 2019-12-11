% close all
% clear all
% clc
%% System Parametes from Data sheet
Kt =  53.4e-3;                          % Torque Constant [Nm/A] 
rg = 100;                               % Gear Ratio of harmonic drive
Jbr = 0.5*0.09*( (0.06^2) + (0.076^2) ); % Inertia of bearing
Jtot =  (9.1364911e-01  * 1e-6) +  (Jbr/(rg^2)) ; % Inertia of motor rotor + harmonic drive , kilogram metre squared [kg. m2]
Jtot = 1.8*Jtot;
Jtot = Jtot + ( 1210e-7);
Jtot = 1.3*Jtot;
Jsint = 1.1e-4 ;                    % Inertia of Internal Torque Sensor Ring + Metal Coupling  
Jsout = 9.58e-4 ;                   % Inertia of External Torque Sensor Ring
kg = 0.84e4;                        % Harmonic Drive Stiffness  [Nm/rad]
kg = 1.15*kg;
%ks = 8.1853e3;                     % Torque Sensor Stiffness  [Nm/rad]
Jl =  0.04 + Jsout  ;               % load Inertia [Kg.m2]
ds = 0;                             % Torque Sensor Damping Ratio [Nm.Sec/rad]
Vel_mot_nom = 391;                  % nominal motor speed [rad/sec]
T_sweep = 75;                       % Duration of input chirp signal (sweep) [sec]
f0 = 0.1;                           % Chirp signal initial frequency [Hz]
f1 = 75;                            % Final Frequency [Hz]
f_log = 1000;                       % Sampling frequency
% dgr = 25;                         % Harmonic Drive Damping Ratio [Nm.Sec/rad]
% dgs = 7;                          % Damping Ratio between torque sensor and harmonic drive [Nm.Sec/rad]
%%%%%%%%%%%%%% GAMC modifications
%this matches resonance from experimental response
%ks = 8.1853e4;
ks = 8.1853e4*1.4; 
Jl = 1.2 * Jl;
%Erfan's dgr value is far too high and the simulated motor velocity reflected at output of gearbox does not match experimental response
% dgr=0.65;           
% dgs = 6;    
dgr= 15;           
dgs = 1;   

%%
f_log = 1000;                     % Sampling frequency
I_cur = 2.5;                        % Motor current
T_sweep = 75;                      % Duration of input chirp signal (sweep) [sec]
f0 = 0.1;                           % Chirp signal initial frequency [Hz]
f1 = 75;                           % Final Frequency [Hz]
%%
sim('sim_sim')
%%
time_sim = sim_time.Data;
trq_sim = reshape( sim_inpt.Data , size( sim_time.Data) );
trq_mes_sim = reshape( sim_oupt.Data , size( sim_time.Data) );

sys_sim_input  =  trq_sim;          % motor torque
sys_sim_output =  trq_mes_sim;
desiredFs = f_log;
[Txy_sim,F_sim] = tfestimate( sys_sim_input , sys_sim_output , 1024 , [] , [] , desiredFs);
systfest_sim = frd(Txy_sim,2*pi*F_sim);

fignum=11;
%%
figure(fignum)
hold on;
h_sim = bodeplot(systfest_sim,'r',systfest_sim.Frequency );
setoptions(h_sim,'Xlim',[1,75],'FreqUnits','Hz')
grid on;
%%
%compare torques sensor
figure(fignum+1)
hold on
plot(time_sim,trq_mes_sim,':b' );grid;shg
title('torque sensor')
%
%compare motor velocities to ensure dgr is a suitable value
vel_sim = reshape( sim_oupt_vel.Data , size( sim_time.Data) );
figure(fignum+2)
grid on;shg
hold on
plot(time_sim , vel_sim ,'c')
title('motor velocities reflected at gearbox output')


