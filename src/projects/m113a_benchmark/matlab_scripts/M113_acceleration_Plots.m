%--------------------------------------------------------------------------
% Mike Taylor 
% M113_ACCELERATION Plot script
%--------------------------------------------------------------------------

clc
clear all
close all

%--------------------------------------------------------------------------

data = load('D:\ChronoEngine\Build_ChronoVehicleTests\bin\M113_ACCELERATION\output.dat','-ascii');

%--------------------------------------------------------------------------
% Sort the Simulation Data into Channels
%--------------------------------------------------------------------------

time = data(:,1);
steering = data(:,2);
throttle = data(:,3);
braking = data(:,4);
LeftTrackAngVel = data(:,5);
RightTrackAngVel = data(:,6);
PTMotorAngVel = data(:,7);
PTMotorTrq = data(:,8);
ChassisPos = data(:,9:11);
ChassisVel = data(:,12:14);
ChassisAccel = data(:,15:17);
ChassisAccel_ChronoFiltered = data(:,18:20);
DrivePos = data(:,21:23);
DriveVel = data(:,24:26);
DriveAccel = data(:,27:29);
DriveAccel_ChronoFiltered = data(:,30:32);
ChassisFLPos = data(:,33:35);
ChassisFRPos = data(:,36:38);
ChassisRLPos = data(:,39:41);
ChassisRRPos = data(:,42:44);

%--------------------------------------------------------------------------
% Generate a Low Pass Filter Object for processing some of the data
%--------------------------------------------------------------------------
% All frequency values are in Hz.
Fs = 1/mean(diff(data(:,1)));  % Sampling Frequency
N  = 6;  % Order
Fc = 5;  % Cutoff Frequency
% Construct an FDESIGN object and call its BUTTER method.
h  = fdesign.lowpass('N,F3dB', N, Fc, Fs);
Hd = design(h, 'butter');

%--------------------------------------------------------------------------
% Calculated Channels
%--------------------------------------------------------------------------

VehicleSpeed = sqrt(ChassisVel(:,1).^2+ChassisVel(:,2).^2);

%--------------------------------------------------------------------------
% Generate Plots
%--------------------------------------------------------------------------

figure();
plot(time,VehicleSpeed,'linewidth',3);
grid on;
xlabel('Time (s)');
ylabel('Vehicle Ground Speed (m/s)');
title({'Chrono::Vehicle M113 - Acceleration Test - 0 Degree slope - Vehicle Speed vs. Time, hard surface, mu= 0.8';...
    ['Maximum Speed: ',num2str(max(VehicleSpeed)),'m/s']});
set(gca(),'FontSize',16);

figure();
plot(time,VehicleSpeed*2.23694,'linewidth',3);
grid on;
xlabel('Time (s)');
ylabel('Vehicle Ground Speed (mph)');
title({'Chrono::Vehicle M113 - Acceleration Test - 0 Degree slope - Vehicle Speed vs. Time, hard surface, mu= 0.8';...
    ['Maximum Speed: ',num2str(max(VehicleSpeed*2.23694)),'mph']});
set(gca(),'FontSize',16);

figure();
plot(time,LeftTrackAngVel,'linewidth',3);
hold on;
plot(time,RightTrackAngVel,'linewidth',3);
grid on;
xlabel('Time (s)');
ylabel('Track Sprocket Angular Velocity (rad/s)');
title('Chrono::Vehicle M113 - Acceleration Test - 0 Degree slope - Sprocket Angular Velocities, hard surface, mu= 0.8');
set(gca(),'FontSize',16);
