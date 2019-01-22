clear all; close all; clc;
dat1 =load('/home/nkapania/catkin_ws/src/harpagornis/bags/out4.mat');
dat1.lane = 3;

dat2 =load('/home/nkapania/catkin_ws/src/harpagornis/bags/out3.mat');
dat2.lane = 2;

laneWidth = 3.7;
width = 1.5;

dat1.vehicleX = dat1.lane * laneWidth + laneWidth /2 - width / 2;
dat2.vehicleX = dat2.lane * laneWidth + laneWidth /2 - width / 2;

pedestrianY = 28.0;

%%
N1 = length(dat1.inCrosswalk_bool);
N2 = length(dat2.inCrosswalk_bool);

dat1.pedXY = zeros(N1,2);
dat1.vehXY = zeros(N1,2);
dat1.distXY = zeros(N1,1);

for i = 1:N1
    dat1.pedXY(i,:) = [dat1.xP(i) pedestrianY];
    dat1.vehXY(i,:) = [dat1.vehicleX dat1.xV(i)];
    dat1.distXY(i) = norm(dat1.pedXY(i,:) - dat1.vehXY(i,:));
end


dat2.pedXY = zeros(N2,2);
dat2.vehXY = zeros(N2,2);
dat2.distXY = zeros(N2,1);

for i = 1:N2
    dat2.pedXY(i,:)  = [dat2.xP(i) pedestrianY];
    dat2.vehXY(i,:)  = [dat2.vehicleX dat2.xV(i)];
    dat2.distXY(i) = norm(dat2.pedXY(i,:) - dat2.vehXY(i,:));
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




%%
subplot(3,2,1)
grid on; hold on; 
title('Rightmost Lane','FontSize',18)
scatter(dat1.acceptedGapDownsample, dat1.minDistance,'k*')
ylim([0, 13]);
xlim([0, 9]);
ylabel('Minimum Distance to Pedestrian(m)')


subplot(3,2,2)
grid on; hold on; 
title('Second to Right Lane','FontSize',18)
scatter(dat2.acceptedGapDownsample, dat2.minDistance,'ro')
ylim([0, 9]);
xlim([0, 9]);

subplot(3,2,3)
scatter(dat1.acceptedGapDownsample, dat1.avgVelocity,'k*')
grid on; hold on; 
ylim([0, 5]);
xlim([0, 9]);
ylabel('Average Velocity (m/s)','FontSize',18)

subplot(3,2,4)
scatter(dat2.acceptedGapDownsample, dat2.avgVelocity,'ro')
grid on; hold on; 

ylim([0, 5]);
xlim([0, 9]);

subplot(3,2,5)
grid on; hold on; 
scatter(dat1.acceptedGapDownsample, dat1.maxAccel,'k*')
ylim([0, 9]);
xlim([0, 9]);
xlabel('Pedestrian Accepted Gap (s)')
ylabel('Maximum Acceleration (m/s2)')

subplot(3,2,6)
grid on; hold on; 
scatter(dat2.acceptedGapDownsample, dat2.maxAccel,'ro')
ylim([0, 9]);
xlim([0, 9]);
ylabel('Maximum Acceleration (m/s2)')
xlabel('Pedestrian Accepted Gap (s)')

%%
% ind2 = find(velocity_mps == 0);
% stoppedDistances = distance_m(ind2);
% histogram(stoppedDistances);

