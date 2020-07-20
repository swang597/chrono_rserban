%--------------------------------------------------------------------------
% Mike Taylor 
% M113_HALFROUNDOBSTACLE Combined Plot script
%--------------------------------------------------------------------------

clc
clear all
close all

%--------------------------------------------------------------------------

Speeds = 1:.25:14;
Bumps = 4:2:12;
Peak_Accel = zeros(length(Speeds),length(Bumps),2);

for s = 1:length(Speeds)
    for b = 1:length(Bumps)
%--------------------------------------------------------------------------

data = load(['D:\ChronoEngine\Build_ChronoVehicleTests\bin\M113_HALFROUNDOBSTACLE\Euler\output_',num2str(Speeds(s),'%4.2f'),'_',num2str(Bumps(b)),'.dat'],'-ascii');

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
Driver_vert_accel_from_speed = [0;diff(DriveVel(:,3))./diff(time)];

%--------------------------------------------------------------------------
start_idx = find(time>=5,1,'first');
Peak_Accel(s,b,1) = max(abs(DriveAccel_ChronoFiltered(start_idx:end,3)));
startpos_idx = find(ChassisPos(:,1)>=110,1,'first');
endpos_idx = find(ChassisPos(:,1)<=120,1,'last');
Peak_Accel(s,b,2) = mean(VehicleSpeed(startpos_idx:endpos_idx));

% if(b == length(Bumps))
%     figure();
%     plot(time(start_idx:end),DriveAccel(start_idx:end,3)/9.80665)
%     hold on
%     plot(time(start_idx:end),DriveAccel_ChronoFiltered(start_idx:end,3)/9.80665)
%     title(['Speed: ',num2str(Peak_Accel(s,b,2)),' m/s - Bump: ',num2str(Bumps(b)),' in']);
% end

    end
end

%--------------------------------------------------------------------------
% Generate Plots
%--------------------------------------------------------------------------

legend_text = cell(length(Bumps),1);
figure();
for b = 1:length(Bumps)
    plot(Peak_Accel(:,b,2)*2.23694,Peak_Accel(:,b,1)/9.80665,'-o','linewidth',3);
    hold on
    legend_text(b) = {[num2str(Bumps(b)),' in']};
end
grid on;
xlabel('Speed (mph)');
ylabel('Peak Driver Vertical Acceleration (g)');
title('Chrono::Vehicle M113 - Event 4b - Half Round Obstacles - Vehicle Speed vs. Peak Driver Vertical Acceleration');
set(gca(),'FontSize',16);
legend(legend_text);
grid minor;
plot([0,25],[2.5,2.5],'k:');
