%--------------------------------------------------------------------------
% Mike Taylor 
% M113_FUELECONOMY Plot script
%--------------------------------------------------------------------------

clc
clear all
close all

%--------------------------------------------------------------------------

data = load('D:\ChronoEngine\Build_ChronoVehicleTests\bin\M113_FUELECONOMY\output.dat','-ascii');

%--------------------------------------------------------------------------
% Sort the Simulation Data into Channels
%--------------------------------------------------------------------------

data = data(1:floor(size(data,1)*.77),:);

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
Power = PTMotorAngVel.*PTMotorTrq;
Energy = cumsum(Power)*mean(diff(time));
Distance = cumsum(VehicleSpeed)*mean(diff(time));
MotionResCo = Energy./(Distance*((78.57+10.14)*1000));

%--------------------------------------------------------------------------
% Generate Plots
%--------------------------------------------------------------------------

figure();
plot(ChassisPos(:,1),ChassisPos(:,2),'linewidth',3);
grid on;
xlabel('Chassis CG X Position (m)');
ylabel('Chassis CG Y Position (m)');
title('Chrono::Vehicle M113 - Event 7a - Fuel Economy - Vehicle CG X Position vs. Vehicle CG Y Position');
set(gca(),'FontSize',16);
axis image


figure();
hold on
patch([-500,-500,1000,1000],[0,600,600,0],[240,240,240]/256)
patch([-500,-500,1000,1000],[200,400,400,200],[220,220,220]/256)
colormap cool;
scatter(ChassisPos(:,2),ChassisPos(:,1),50,PTMotorAngVel.*PTMotorTrq/1000);
grid on;
xlabel('Chassis CG Y Position (m)');
ylabel('Chassis CG X Position (m)');
title('Chrono::Vehicle M113 - Event 7a - Fuel Economy - Vehicle CG Position vs. Engine Power');
set(gca(),'FontSize',16);
axis image
h_cb = colorbar();
ylabel(h_cb,'Power (kW)');
set(gca(), 'xdir','reverse')


figure();
hold on
patch([-500,-500,1000,1000],[0,600,600,0],[240,240,240]/256)
patch([-500,-500,1000,1000],[200,400,400,200],[220,220,220]/256)
colormap cool;
scatter(ChassisPos(:,2),ChassisPos(:,1),50,VehicleSpeed);
grid on;
xlabel('Chassis CG Y Position (m)');
ylabel('Chassis CG X Position (m)');
title('Chrono::Vehicle M113 - Event 7a - Fuel Economy - Vehicle CG Position vs. Vehicle Speed');
set(gca(),'FontSize',16);
axis image
h_cb = colorbar();
ylabel(h_cb,'Vehicle Speed (m/s)');
set(gca(), 'xdir','reverse')


figure();
hold on
patch([-500,-500,1000,1000],[0,600,600,0],[240,240,240]/256)
patch([-500,-500,1000,1000],[200,400,400,200],[220,220,220]/256)
colormap cool;
scatter(ChassisPos(:,2),ChassisPos(:,1),50,Energy/1e6);
grid on;
xlabel('Chassis CG Y Position (m)');
ylabel('Chassis CG X Position (m)');
title(['Chrono::Vehicle M113 - Event 7a - Fuel Economy - Vehicle CG Position vs. Energy Used - Total: ',num2str(Energy(end)/1e6),'MJ']);
set(gca(),'FontSize',16);
axis image
h_cb = colorbar();
ylabel(h_cb,'Energy Used by the Powertrain (MJ)');
set(gca(), 'xdir','reverse')


figure();
hold on
patch([-500,-500,1000,1000],[0,600,600,0],[240,240,240]/256)
patch([-500,-500,1000,1000],[200,400,400,200],[220,220,220]/256)
colormap cool;
scatter(ChassisPos(:,2),ChassisPos(:,1),50,Distance);
grid on;
xlabel('Chassis CG Y Position (m)');
ylabel('Chassis CG X Position (m)');
title('Chrono::Vehicle M113 - Event 7a - Fuel Economy - Vehicle CG Position vs. Distance');
set(gca(),'FontSize',16);
axis image
h_cb = colorbar();
ylabel(h_cb,'Distance (m)');
set(gca(), 'xdir','reverse')


figure();
hold on
patch([-500,-500,1000,1000],[0,600,600,0],[240,240,240]/256)
patch([-500,-500,1000,1000],[200,400,400,200],[220,220,220]/256)
colormap cool;
scatter(ChassisPos(:,2),ChassisPos(:,1),50,MotionResCo);
grid on;
xlabel('Chassis CG Y Position (m)');
ylabel('Chassis CG X Position (m)');
title('Chrono::Vehicle M113 - Event 7a - Fuel Economy - Vehicle CG Position vs. Motion Resistance Coefficent');
set(gca(),'FontSize',16);
axis image
h_cb = colorbar();
ylabel(h_cb,' Motion Resistance Coefficent');
set(gca(), 'xdir','reverse')

figure();
plot(time,MotionResCo,'linewidth',3);
set(gca(),'yscale','log');
grid on;
xlabel('Time (s)');
ylabel('Motion Resistance Coefficent (Cumulative Energy/(Cumulative Distance x GVW))');
title(['Chrono::Vehicle M113 - Event 7a - Fuel Economy - Time vs. Motion Resistance Coefficent - Final Value: ',num2str(MotionResCo(end))]);
set(gca(),'FontSize',16);

figure();
plot(Distance,MotionResCo,'linewidth',3);
set(gca(),'yscale','log');
grid on;
xlabel('Distance (m)');
ylabel('Motion Resistance Coefficent (Cumulative Energy/(Cumulative Distance x GVW))');
title(['Chrono::Vehicle M113 - Event 7a - Fuel Economy - Distance vs. Motion Resistance Coefficent - Final Value: ',num2str(MotionResCo(end))]);
set(gca(),'FontSize',16);


