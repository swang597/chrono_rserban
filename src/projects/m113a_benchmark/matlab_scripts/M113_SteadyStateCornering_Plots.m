%--------------------------------------------------------------------------
% Mike Taylor 
% M113_STEADYSTATECORNERING Plot script
%--------------------------------------------------------------------------

clc
clear all
close all

%--------------------------------------------------------------------------

data = load('D:\ChronoEngine\Build_ChronoVehicleTests\bin\M113_STEADYSTATECORNERING\output.dat','-ascii');

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
if(mean(LeftTrackAngVel)<mean(RightTrackAngVel)) %CW
    %Negative since Track Ang Velocities are negative for forward motion
    TurnAngle = -(LeftTrackAngVel-RightTrackAngVel)*0.214./VehicleSpeed;
else %CCW
    TurnAngle = -(RightTrackAngVel-LeftTrackAngVel)*0.214./VehicleSpeed;
end

Roll_rad = atan2(ChassisFLPos(:,3)-ChassisFRPos(:,3),sqrt((ChassisFLPos(:,1)-ChassisFRPos(:,1)).^2+(ChassisFLPos(:,2)-ChassisFRPos(:,2)).^2));
Roll_deg = Roll_rad*180/pi();

%--------------------------------------------------------------------------
% Generate Plots
%--------------------------------------------------------------------------

figure();
plot(time,VehicleSpeed*2.23694,'linewidth',3);
grid on;
xlabel('Time (s)');
ylabel('Vehicle Ground Speed (mph)');
set(gca(),'FontSize',16);
if(mean(LeftTrackAngVel)<mean(RightTrackAngVel))
    title('Chrono::Vehicle M113 - Event 1b - Steady State Cornering: Clockwise - Vehicle Speed vs. Time, hard surface, mu= 0.8');
else
    title('Chrono::Vehicle M113 - Event 1b - Steady State Cornering: Counter-Clockwise - Vehicle Speed vs. Time, hard surface, mu= 0.8');    
end

figure();
subplot(2,1,1);
plot(time,TurnAngle,'linewidth',3);
grid on;
xlabel('Time (s)');
ylabel('Kinematic Turning Ratio');
if(mean(LeftTrackAngVel)<mean(RightTrackAngVel))
    title('Chrono::Vehicle M113 - Event 1b - Steady State Cornering: Clockwise - Kinematic Turning Ratio, hard surface, mu= 0.8');
else
    title('Chrono::Vehicle M113 - Event 1b - Steady State Cornering: Counter-Clockwise - Kinematic Turning Ratio, hard surface, mu= 0.8');    
end
set(gca(),'FontSize',16);
ylim([-1,1]);
subplot(2,1,2);
plot(time,Roll_deg,'linewidth',3);
grid on;
xlabel('Time (s)');
ylabel('Roll Angle (deg)');
title('Chrono M113 - Event 1b - Steady State Cornering - Roll Angle');
set(gca(),'FontSize',16);
ylim([-1,1]);
if(mean(LeftTrackAngVel)<mean(RightTrackAngVel))
    title('Chrono::Vehicle M113 - Event 1b - Steady State Cornering: Clockwise - Roll Angle, hard surface, mu= 0.8');
else
    title('Chrono::Vehicle M113 - Event 1b - Steady State Cornering: Counter-Clockwise - Roll Angle, hard surface, mu= 0.8');    
end

idx_start = 1;%find(time>1,1,'first');
idx_end = length(time);%find(time>39.9,1,'first');
figure();
plot(ChassisPos(idx_start:idx_end,1)/0.3048,ChassisPos(idx_start:idx_end,2)/0.3048,'linewidth',3);
hold on
plot(ChassisFLPos(idx_start:idx_end,1)/0.3048,ChassisFLPos(idx_start:idx_end,2)/0.3048,'linewidth',3);
plot(ChassisFRPos(idx_start:idx_end,1)/0.3048,ChassisFRPos(idx_start:idx_end,2)/0.3048,'linewidth',3);
plot(ChassisRLPos(idx_start:idx_end,1)/0.3048,ChassisRLPos(idx_start:idx_end,2)/0.3048,'linewidth',3);
plot(ChassisRRPos(idx_start:idx_end,1)/0.3048,ChassisRRPos(idx_start:idx_end,2)/0.3048,'linewidth',3);
xlabel('x position (ft)');
ylabel('y position (ft)');
%axis image
grid on
legend('Chassis CG','Front Left','Front Right','Rear Left','Rear Right');
axis image
if(mean(LeftTrackAngVel)<mean(RightTrackAngVel))
    title('Chrono::Vehicle M113 - Event 1b - Steady State Cornering: Clockwise - Vehicle Position, hard surface, mu= 0.8');
else
    title('Chrono::Vehicle M113 - Event 1b - Steady State Cornering: Counter-Clockwise - Vehicle Position, hard surface, mu= 0.8');    
end
set(gca(),'FontSize',16);
