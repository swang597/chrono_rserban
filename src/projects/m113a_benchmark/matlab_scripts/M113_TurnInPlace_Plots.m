%--------------------------------------------------------------------------
% Mike Taylor 
% M113_TURNINPLACE Plot script
%--------------------------------------------------------------------------

clc
clear all
close all

%--------------------------------------------------------------------------

data = load('D:\ChronoEngine\Build_ChronoVehicleTests\bin\M113_TURNINPLACE\output.dat','-ascii');

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
title('Chrono::Vehicle M113 - Event 1a - Neutral Axis Spin - Vehicle Speed vs. Time');
set(gca(),'FontSize',16);

figure();
plot(time,VehicleSpeed*2.23694,'linewidth',3);
grid on;
xlabel('Time (s)');
ylabel('Vehicle Ground Speed (mph)');
title('Chrono::Vehicle M113 - Event 1a - Neutral Axis Spin - Vehicle Speed vs. Time');
set(gca(),'FontSize',16);


idx_start = find(time>15,1,'first');
idx_end = find(time>20,1,'first');
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

ParOrigin = CircleFitByTaubin(ChassisPos(idx_start:idx_end,1:2))/0.3048
ParFL = CircleFitByTaubin(ChassisFLPos(idx_start:idx_end,1:2))/0.3048
ParFR = CircleFitByTaubin(ChassisFRPos(idx_start:idx_end,1:2))/0.3048
ParRL = CircleFitByTaubin(ChassisRLPos(idx_start:idx_end,1:2))/0.3048
ParRR = CircleFitByTaubin(ChassisRRPos(idx_start:idx_end,1:2))/0.3048
MaxRadius = max([ParOrigin(3),ParFL(3),ParFR(3),ParRL(3),ParRR(3)])

rectangle('position',[ParOrigin(1)-ParOrigin(3),ParOrigin(2)-ParOrigin(3),ParOrigin(3)*2,ParOrigin(3)*2],...
    'curvature',[1,1],'linestyle',':','edgecolor','k','linewidth',3);
rectangle('position',[ParFL(1)-ParFL(3),ParFL(2)-ParFL(3),ParFL(3)*2,ParFL(3)*2],...
    'curvature',[1,1],'linestyle',':','edgecolor','k','linewidth',3);
rectangle('position',[ParFR(1)-ParFR(3),ParFR(2)-ParFR(3),ParFR(3)*2,ParFR(3)*2],...
    'curvature',[1,1],'linestyle',':','edgecolor','k','linewidth',3);
rectangle('position',[ParRL(1)-ParRL(3),ParRL(2)-ParRL(3),ParRL(3)*2,ParRL(3)*2],...
    'curvature',[1,1],'linestyle',':','edgecolor','k','linewidth',3);
rectangle('position',[ParRR(1)-ParRR(3),ParRR(2)-ParRR(3),ParRR(3)*2,ParRR(3)*2],...
    'curvature',[1,1],'linestyle',':','edgecolor','k','linewidth',3);
set(gca(),'FontSize',16)

if(mean(LeftTrackAngVel)<mean(RightTrackAngVel))
    title({'Chrono::Vehicle M113 - Event 1a - Neutral Axis Spin: Clockwise, hard surface, mu= 0.8';...
        ['Maximum Diameter of Chassis Outermost Point: ',num2str(2*MaxRadius),'ft']});
else
    title({'Chrono::Vehicle M113 - Event 1a - Neutral Axis Spin: Counter-Clockwise, hard surface, mu= 0.8';...
        ['Maximum Diameter of Chassis Outermost Point: ',num2str(2*MaxRadius),'ft']});    
end


plot([ChassisFLPos(idx_start,1)/0.3048,...
      ChassisFRPos(idx_start,1)/0.3048,...
      ChassisRRPos(idx_start,1)/0.3048,...
      ChassisRLPos(idx_start,1)/0.3048,...
      ChassisFLPos(idx_start,1)/0.3048],...
      [ChassisFLPos(idx_start,2)/0.3048,...
      ChassisFRPos(idx_start,2)/0.3048,...
      ChassisRRPos(idx_start,2)/0.3048,...
      ChassisRLPos(idx_start,2)/0.3048,...
      ChassisFLPos(idx_start,2)/0.3048],'k:');
  
 