% close all
% clear all
% clc
%% System Parametes from Data sheet
Kt =  53.4e-3;                          % Torque Constant [Nm/A] 
rg = 100;                               % Gear Ratio of harmonic drive
Jbr = 0.5*0.09*( (0.06^2) + (0.076^2)); % Inertia of bearing
Jtot =  (9.1364911e-01  * 1e-6) +  (Jbr/(rg^2)) ; % Inertia of motor rotor + harmonic drive , kilogram metre squared [kg. m2]
Jtot = 1.55*Jtot;
Jtot = Jtot + ( 1210e-7);
Jtot = 1.75*Jtot;
Jsint = 1.1e-4 ;                    % Inertia of Internal Torque Sensor Ring + Metal Coupling  
Jsout = 9.58e-4 ;                   % Inertia of External Torque Sensor Ring
kg = 0.84e4;                        % Harmonic Drive Stiffness  [Nm/rad]
kg = 1.15*kg;
%ks = 8.1853e3;                     % Torque Sensor Stiffness  [Nm/rad]
Jl =  0.04 + Jsout  ;               % load Inertia [Kg.m2]
ds = 0;                             % Torque Sensor Damping Ratio [Nm.Sec/rad]
Vel_mot_nom = 391;                  % nominal motor speed [rad/sec]
T_sweep = 75;                      % Duration of input chirp signal (sweep) [sec]
f0 = 0.1;                           % Chirp signal initial frequency [Hz]
f1 = 75;                            % Final Frequency [Hz]
f_log = 1000;                        % Sampling frequency
% dgr = 25;                           % Harmonic Drive Damping Ratio [Nm.Sec/rad]
% dgs = 7;                            % Damping Ratio between torque sensor and harmonic drive [Nm.Sec/rad]
%%%%%%%%%%%%%% GAMC modifications
%this matches resonance from experimental response
ks = 8.1853e4*1.4;   
ks = 1*ks;
Jl = 1.2 * Jl;
%Erfan's dgr value is far too high and the simulated motor velocity reflected at output of gearbox does not match experimental response
% dgr=0.65;           
% dgs = 6;    
dgr= .05;           
dgs = 3.5; 
%%
Data_exp = csvread('3_75_75.csv' , 1 , 0 );  % Reading Experimental Data
Data_exp(end , :) = [];

%%
samples = 1:length( Data_exp(:,1) ) ;
tempo = Data_exp( samples , 1);
tempo=tempo-tempo(1);
in_cur = Data_exp( samples , 2);
out_cur = Data_exp( samples , 3);
out_vel = Data_exp( samples , 4);
out_trq = Data_exp( samples , 5);
%%
originalFs = 1 / mean ( diff( tempo ) )    % Real Sampling Frequency [Hz]

%% Removing Torque Sensor Off_set
out_trq_off = out_trq - mean( out_trq );

%%  Resample Data %%%%%%%%%%%%%%%%%%

desiredFs = 1000;
%desiredFs = originalFs + (.0*originalFs);
[p,q] = rat(desiredFs / originalFs);
%[p,q] = rat(originalFs / originalFs);

cur_mes_rspl = resample( out_cur , p , q );     % Resample Motor Current
vel_mes_rspl = resample( out_vel , p , q );     % Resample Motor Velocity
trq_mes_rspl = resample( out_trq_off , p , q );     % Resample Sensor Torque

time_rspl = (0:numel(cur_mes_rspl)-1)/desiredFs;
t_dur = tempo(end);            % Duration Time of Experiment [sec]

%% LOW PASS FILTER 

Fs = desiredFs;       % Sampling Frequency [Hz]
%Fs = originalFs;
Fc = 75;               % Cut of Frequency [Hz]
% 
[b,a] = butter( 6 , Fc/(Fs/2) );                 % Butterworth filter of order 6
trq_flt = filtfilt( b , a , trq_mes_rspl );     % Will be the filtered measured torque signal
cur_flt = filtfilt( b , a , cur_mes_rspl );     % Will be the filtered measured current signal

% sys_exp_input  =  cur_flt .* Kt .* rg;
% sys_exp_output =   trq_flt;

%sys_exp_input  =  cur_mes_rspl .* Kt .* rg;    %this is wrong, rg is part of the system hardware you do not need to multiply by rg here
sys_exp_input  =  cur_mes_rspl .* Kt;          %motor torque

sys_exp_output =  trq_mes_rspl;

[Txy_exp,F_exp] = tfestimate( sys_exp_input , -sys_exp_output , 1024 , [] , [] , desiredFs);
systfest_exp = frd(Txy_exp,2*pi*F_exp);

fignum=11;

figure(fignum)
hold on;
h_exp = bodeplot(systfest_exp,'r',systfest_exp.Frequency );
setoptions(h_exp,'Xlim',[1,75],'FreqUnits','Hz')
grid on;

%%
Torque_input_ref = [ time_rspl' , ( cur_mes_rspl .* Kt) ];  % Input torque is calculated as motor curret * torque constant

sim('sim_exp_sim')
%sim('sim_sim_spl_exp')

trq_sen_sim =  sim_oupt.Data;
trq_mot_exp = sim_inpt.Data;
%[Txy_exp_sim,F_exp_sim] = tfestimate(  trq_mot_exp .* rg , trq_sen_sim , 1024 , [] , [] , desiredFs); %this is wrong, rg is part of the simulink block diagram you do not need to multiply by rg here
[Txy_exp_sim,F_exp_sim] = tfestimate(  trq_mot_exp , trq_sen_sim , 1024 , [] , [] , desiredFs);
systfest_exp_sim = frd(Txy_exp_sim,2*pi*F_exp_sim);

figure(fignum)
hold on;
h_exp_sim = bodeplot(systfest_exp_sim,'k',systfest_exp_sim.Frequency );
setoptions(h_exp_sim,'Xlim',[1,75],'FreqUnits','Hz')
grid on;
hold
legend('exp frequency response', 'sim frequency response')

%compare motor velocities to ensure dgr is a suitable value
figure(fignum+1)
plot(time_rspl,vel_mes_rspl );grid;shg
hold
plot(tempo,out_vel,':r');shg
plot(sim_time.Data,sim_oupt_vel.Data,'--c');shg
hold
legend('exp data resampled','exp data','sim data')
title('motor velocities reflected at gearbox output')

%compare torques sensor
figure(fignum+2)
plot(time_rspl,trq_mes_rspl );grid;shg
hold
plot(tempo,out_trq_off,'r');shg
plot(sim_time.Data,-sim_oupt.Data,':c');shg
hold
legend('exp data resampled','exp data','sim data')
title('torque sensor')
