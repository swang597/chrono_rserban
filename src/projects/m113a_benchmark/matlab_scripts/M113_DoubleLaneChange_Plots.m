%--------------------------------------------------------------------------
% Mike Taylor 
% M113_DOUBLELANECHANGE Plot script
%--------------------------------------------------------------------------

clc
clear all
close all

%--------------------------------------------------------------------------

data = load('D:\ChronoEngine\Build_ChronoVehicleTests\bin\M113_DOUBLELANECHANGE\output.dat','-ascii');

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

%surface_type = 'hard surface, mu= 0.8'
%event_id = '1c';
surface_type = 'hard gravel surface, mu= 0.5';
event_id = '1d';

figure();
subplot(2,1,1);
[hAx,hLine1,hLine2] = plotyy(ChassisPos(:,1)/0.3048,VehicleSpeed*2.23694,ChassisPos(:,1)/0.3048,steering);
set(hLine1,'Linewidth',3);
set(hLine2,'Linewidth',1);
grid on;
xlabel('Chassis CG X Position (ft)');
ylabel(hAx(1),'Vehicle Ground Speed (mph)');
ylabel(hAx(2),'Steering Signal (bounds +/-1)');
set(hAx(1),'FontSize',16);
set(hAx(2),'FontSize',16);
title(['Chrono::Vehicle M113 - Event ',event_id,' - Double Lane Change - Vehicle Speed vs. Time, ',surface_type]);
xlim(hAx(1),[-100,400]);
xlim(hAx(2),[-100,400]);
set(hAx(1),'YTick',0:5:40)

idx_start = 1;%find(time>1,1,'first');
idx_end = length(time);%find(time>39.9,1,'first');
%figure();
subplot(2,1,2);
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
%axis image
title(['Chrono::Vehicle M113 - Event ',event_id,' - Double Lane Change - Vehicle Position, ',surface_type]);
set(gca(),'FontSize',16);

VW = 106 * 0.0254;
Leff = 191.7 * 0.0254;
W1 = 1.1*VW+.25;
W2 = 1.2*VW+.25;

%Starting gates
plot([0,15]/0.3048,[-W1/2,-W1/2]/0.3048,'k-');
plot([0,15]/0.3048,[ W1/2, W1/2]/0.3048,'k-');
plot([0,15]/0.3048,[0,0]/0.3048,'k:');

%Mid gates
plot([15+24+Leff,15+24+Leff+25]/0.3048,[3.5-W1/2,3.5-W1/2]/0.3048,'k-');
plot([15+24+Leff,15+24+Leff+25]/0.3048,[3.5-W1/2+W2,3.5-W1/2+W2]/0.3048,'k-');
plot([15+24+Leff,15+24+Leff+25]/0.3048,[3.5-W1/2+W2/2,3.5-W1/2+W2/2]/0.3048,'k:');

%Ending gates
plot([15+2*(24+Leff)+25,2*15+2*(24+Leff)+25]/0.3048,[-W1/2,-W1/2]/0.3048,'k-');
plot([15+2*(24+Leff)+25,2*15+2*(24+Leff)+25]/0.3048,[ W1/2, W1/2]/0.3048,'k-');
plot([15+2*(24+Leff)+25,2*15+2*(24+Leff)+25]/0.3048,[0,0]/0.3048,'k:');

xlim([-100,400]);


%Plot Bezier Path Curves
pnts=[   -125.0000       0.0000       0.1000    -125.0000       0.0000       0.1000    -110.0000       0.0000       0.1000
	 15.0000       0.0000       0.1000    -110.0000       0.0000       0.1000      29.4346       0.0000       0.1000
	 43.8692       3.6346       0.1000      29.4346       3.6346       0.1000      56.3692       3.6346       0.1000
	 68.8692       3.6346       0.1000      56.3692       3.6346       0.1000      83.3038       3.6346       0.1000	 
     97.7384       0.0000       0.1000      83.3038       0.0000       0.1000     105.2384       0.0000       0.1000	 	 
	150.0000       0.0000       0.1000     105.2384       0.0000       0.1000     150.0000       0.0000       0.1000];

t = 0:.01:1;
pts1 = kron((1-t).^3,pnts(1,1:2)') + kron(3*(1-t).^2.*t,pnts(1,7:8)') + kron(3*(1-t).*t.^2,pnts(2,4:5)') + kron(t.^3,pnts(2,1:2)');
pts2 = kron((1-t).^3,pnts(2,1:2)') + kron(3*(1-t).^2.*t,pnts(2,7:8)') + kron(3*(1-t).*t.^2,pnts(3,4:5)') + kron(t.^3,pnts(3,1:2)');
pts3 = kron((1-t).^3,pnts(3,1:2)') + kron(3*(1-t).^2.*t,pnts(3,7:8)') + kron(3*(1-t).*t.^2,pnts(4,4:5)') + kron(t.^3,pnts(4,1:2)');
pts4 = kron((1-t).^3,pnts(4,1:2)') + kron(3*(1-t).^2.*t,pnts(4,7:8)') + kron(3*(1-t).*t.^2,pnts(5,4:5)') + kron(t.^3,pnts(5,1:2)');
pts5 = kron((1-t).^3,pnts(5,1:2)') + kron(3*(1-t).^2.*t,pnts(5,7:8)') + kron(3*(1-t).*t.^2,pnts(6,4:5)') + kron(t.^3,pnts(6,1:2)');

hold on
plot(pts1(1,:)/0.3048,pts1(2,:)/0.3048,'k--','linewidth',2)
plot(pts2(1,:)/0.3048,pts2(2,:)/0.3048,'k--','linewidth',2)
plot(pts3(1,:)/0.3048,pts3(2,:)/0.3048,'k--','linewidth',2)
plot(pts4(1,:)/0.3048,pts4(2,:)/0.3048,'k--','linewidth',2)
plot(pts5(1,:)/0.3048,pts5(2,:)/0.3048,'k--','linewidth',2)