clear all; close all; clc;
 dat1 =load('/home/nkapania/catkin_ws/src/harpagornis/bags/out1.mat');
 dat1p = load('/home/nkapania/catkin_ws/src/harpagornis/bags/out_pomdp_1.mat');
%dat1 =load('/home/nkapania/catkin_ws/src/harpagornis/bags/out4.mat');
%dat1p = load('/home/nkapania/catkin_ws/src/harpagornis/bags/out_pomdp_4.mat');


dat1.lane = 3;

% dat1 =load('/home/nkapania/catkin_ws/src/harpagornis/bags/out1.mat');
% dat1p = load('/home/nkapania/catkin_ws/src/harpagornis/bags/out_pomdp_1.mat');
dat2 =load('/home/nkapania/catkin_ws/src/harpagornis/bags/out2.mat');
dat2p = load('/home/nkapania/catkin_ws/src/harpagornis/bags/out_pomdp_2.mat');
dat2.lane = 2;

laneWidth = 3.7;
width = 1.5;

dat1.vehicleX = dat1.lane * laneWidth + laneWidth /2 - width / 2;
dat1p.vehicleX = dat1.vehicleX;
dat2.vehicleX = dat2.lane * laneWidth + laneWidth /2 - width / 2;
dat2p.vehicleX = dat2.vehicleX;
pedestrianY = 28.0;

%%
N1 = length(dat1.inCrosswalk_bool);
N1p = length(dat1p.inCrosswalk_bool);
N2 = length(dat2.inCrosswalk_bool);
N2p = length(dat2p.inCrosswalk_bool);

dat1.pedXY = zeros(N1,2);
dat1.vehXY = zeros(N1,2);
dat1.distXY = zeros(N1,1);

dat1p.pedXY = zeros(N1p,2);
dat1p.vehXY = zeros(N1p,2);
dat1p.distXY = zeros(N1p,1);

for i = 1:N1
    dat1.pedXY(i,:) = [dat1.xP(i) pedestrianY];
    dat1.vehXY(i,:) = [dat1.vehicleX dat1.xV(i)];
    dat1.distXY(i) = norm(dat1.pedXY(i,:) - dat1.vehXY(i,:));
end

for i = 1:N1p
    dat1p.pedXY(i,:) = [dat1p.xP(i) pedestrianY];
    dat1p.vehXY(i,:) = [dat1p.vehicleX dat1p.xV(i)];
    dat1p.distXY(i) = norm(dat1p.pedXY(i,:) - dat1p.vehXY(i,:));
end


dat2.pedXY = zeros(N2,2);
dat2.vehXY = zeros(N2,2);
dat2.distXY = zeros(N2,1);

dat2p.pedXY = zeros(N2p,2);
dat2p.vehXY = zeros(N2p,2);
dat2p.distXY = zeros(N2p,1);


for i = 1:N2
    dat2.pedXY(i,:)  = [dat2.xP(i) pedestrianY];
    dat2.vehXY(i,:)  = [dat2.vehicleX dat2.xV(i)];
    dat2.distXY(i) = norm(dat2.pedXY(i,:) - dat2.vehXY(i,:));
end

for i = 1:N2p
    dat2p.pedXY(i,:)  = [dat2p.xP(i) pedestrianY];
    dat2p.vehXY(i,:)  = [dat2p.vehicleX dat2p.xV(i)];
    dat2p.distXY(i) = norm(dat2p.pedXY(i,:) - dat2p.vehXY(i,:));
end

%%

dat1.numTrials = max(dat1.trialNumber);
dat1.avgVelocity = zeros(dat1.numTrials,1);
dat1.maxAccel    = zeros(dat1.numTrials,1); 
dat1.minDistance = zeros(dat1.numTrials,1);
dat1.acceptedGapDownsample = zeros(dat1.numTrials,1);

for i = 1:dat1.numTrials
    ind = find(dat1.trialNumber == i);
    dat1.avgVelocity(i) = mean(dat1.velocity_mps(ind)); 
    dat1.maxAccel(i) = max(abs(dat1.ddxV(ind)));
    dat1.acceptedGapDownsample(i) = mean(dat1.acceptedGap(ind));
    dat1.minDistance(i) = min(dat1.distXY(ind));
end

numGarbageTrials = 3; %first few trials are garbage

dat1p.numTrials = max(dat1p.trialNumber)-numGarbageTrials;
dat1p.avgVelocity = zeros(dat1p.numTrials,1);
dat1p.maxAccel    = zeros(dat1p.numTrials,1); 
dat1p.minDistance = zeros(dat1p.numTrials,1);
dat1p.acceptedGapDownsample = zeros(dat1p.numTrials,1);

for i = 1:dat1p.numTrials
    ind = find(dat1p.trialNumber == i+numGarbageTrials);
    dat1p.avgVelocity(i) = mean(dat1p.velocity_mps(ind)); 
    dat1p.maxAccel(i) = abs(min(dat1p.ddxV(ind))); %different than for hybrid controller b/c accel is 3.0
    dat1p.acceptedGapDownsample(i) = mean(dat1p.acceptedGap(ind));
    dat1p.minDistance(i) = min(dat1p.distXY(ind));
end

%%
dat2.numTrials = max(dat2.trialNumber); 
dat2.avgVelocity = zeros(dat2.numTrials,1);
dat2.maxAccel    = zeros(dat2.numTrials,1); 
dat2.acceptedGapDownsample = zeros(dat2.numTrials,1);
dat2.minDistance = zeros(dat2.numTrials, 1);

for i = 1:dat2.numTrials
    ind = find(dat2.trialNumber == i);
    dat2.avgVelocity(i) = mean(dat2.velocity_mps(ind)); 
    dat2.maxAccel(i) = max(abs(dat2.ddxV(ind)));
    dat2.acceptedGapDownsample(i) = mean(dat2.acceptedGap(ind));
    dat2.minDistance(i) = min(dat2.distXY(ind));
end

numGarbageTrials = 3; %first few trials are garbage

dat2p.numTrials = max(dat2p.trialNumber)-numGarbageTrials;
dat2p.avgVelocity = zeros(dat2p.numTrials,1);
dat2p.maxAccel    = zeros(dat2p.numTrials,1); 
dat2p.minDistance = zeros(dat2p.numTrials,1);
dat2p.acceptedGapDownsample = zeros(dat2p.numTrials,1);

for i = 1:dat2p.numTrials
    ind = find(dat2p.trialNumber == i+numGarbageTrials);
    dat2p.avgVelocity(i) = mean(dat2p.velocity_mps(ind)); 
    dat2p.maxAccel(i) = abs(min(dat2p.ddxV(ind))); %different than for hybrid controller b/c accel is 3.0
    dat2p.acceptedGapDownsample(i) = mean(dat2p.acceptedGap(ind));
    dat2p.minDistance(i) = min(dat2p.distXY(ind));
end

%%
subplot(3,2,1)
grid on; hold on; 
title('Lane A','FontSize',18)
scatter(dat1.acceptedGapDownsample, dat1.minDistance,'k*')
scatter(dat1p.acceptedGapDownsample, dat1p.minDistance,'r*')
ylim([0, 13]);
xlim([0, 9]);
ylabel('Minimum Distance to Pedestrian(m)')


subplot(3,2,2)
grid on; hold on; 
title('Lane B','FontSize',18)
scatter(dat2.acceptedGapDownsample, dat2.minDistance,'k*'); 
scatter(dat2p.acceptedGapDownsample, dat2p.minDistance,'r*')
ylim([0, 9]);
xlim([0, 9]);

subplot(3,2,3)
scatter(dat1.acceptedGapDownsample, dat1.avgVelocity,'k*')
grid on; hold on; 
scatter(dat1p.acceptedGapDownsample, dat1p.avgVelocity,'r*')
ylim([0, 5]);
xlim([0, 9]);
ylabel('Average Velocity (m/s)','FontSize',18)

subplot(3,2,4)
scatter(dat2.acceptedGapDownsample, dat2.avgVelocity,'k*'); 
grid on; hold on; 
scatter(dat2p.acceptedGapDownsample, dat2p.avgVelocity,'r*')
ylim([0, 5]);
xlim([0, 9]);

subplot(3,2,5)
grid on; hold on; 
scatter(dat1.acceptedGapDownsample, dat1.maxAccel,'k*')
scatter(dat1p.acceptedGapDownsample, dat1p.maxAccel,'r*')
ylim([0, 9]);
xlim([0, 9]);
xlabel('Pedestrian Accepted Gap (s)')
ylabel('Maximum Acceleration (m/s2)')

subplot(3,2,6)
grid on; hold on; 
scatter(dat2.acceptedGapDownsample, dat2.maxAccel,'k*')
scatter(dat2p.acceptedGapDownsample, dat2p.maxAccel,'r*')
ylim([0, 9]);
xlim([0, 9]);
ylabel('Maximum Acceleration (m/s2)')
xlabel('Pedestrian Accepted Gap (s)')
legend('Hybrid Controller','POMDP')

%%
% ind2 = find(velocity_mps == 0);
% stoppedDistances = distance_m(ind2);
% histogram(stoppedDistances);

