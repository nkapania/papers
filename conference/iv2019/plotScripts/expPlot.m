clear all; close all; clc;

%% load bags
t1 = load('/home/nkapania/Desktop/190110NRKbags/trial1.mat');
t2 = load('/home/nkapania/Desktop/190110NRKbags/trial2.mat');
t3 = load('/home/nkapania/Desktop/190110NRKbags/trial3.mat');
t4 = load('/home/nkapania/Desktop/190110NRKbags/trial4.mat');
t5 = load('/home/nkapania/Desktop/190110NRKbags/trial5.mat');
t6 = load('/home/nkapania/Desktop/190110NRKbags/trial6.mat');


%% create downsampled versions of actual acceleration
N1 = length(t1.t);
N2 = length(t2.t);
N3 = length(t3.t);
N4 = length(t4.t);
N5 = length(t5.t);
N6 = length(t6.t);

t1.axDS = t1.ax( round(linspace(1, length(t1.ax), N1)) );
t2.axDS = t2.ax( round(linspace(1, length(t2.ax), N2)) );
t3.axDS = t3.ax( round(linspace(1, length(t3.ax), N3)) );
t4.axDS = t4.ax( round(linspace(1, length(t4.ax), N4)) );
t5.axDS = t5.ax( round(linspace(1, length(t5.ax), N5)) );
t6.axDS = t6.ax( round(linspace(1, length(t6.ax), N6)) );

%% get relevant indices
ind1 = intersect(find(t1.distance_m > -5), find(t1.distance_m < 35));
ind2 = intersect(find(t2.distance_m > -5), find(t2.distance_m < 35));
ind3 = intersect(find(t3.distance_m > -5), find(t3.distance_m < 35));
ind4 = intersect(find(t4.distance_m > -5), find(t4.distance_m < 35));
ind5 = intersect(find(t5.distance_m > -5), find(t5.distance_m < 35));
ind6 = intersect(find(t6.distance_m > -5), find(t6.distance_m < 35));


%% Plot

%Generate offset

offset = 5.5;

subplot(6, 2, 1)
plot(-t1.distance_m(ind1)+offset, t1.velocity_mps(ind1),'LineWidth',2,'FontSize',12)
title('Velocity m/s')
ylabel('Trial 1')
ylim([0,8])
xlim([-30 10])

subplot(6, 2, 2)
plot(-t1.distance_m(ind1)+offset, t1.ddxV(ind1),'LineWidth',2,'FontSize',12)
hold on;
title('Acceleration m/s^2')
plot(-t1.distance_m(ind1)+offset, t1.axDS(ind1),'r','LineWidth',2,'FontSize',12)
xlim([-30 10])


subplot(6, 2, 3)
plot(-t2.distance_m(ind2)+offset, t2.velocity_mps(ind2),'LineWidth',2,'FontSize',12)
ylabel('Trial 2')
ylim([0,8])
xlim([-30 10])


subplot(6, 2, 4)
plot(-t2.distance_m(ind2)+offset, t2.ddxV(ind2),'LineWidth',2,'FontSize',12)
hold on;
plot(-t2.distance_m(ind2)+offset, t2.axDS(ind2),'r','LineWidth',2,'FontSize',12)
xlim([-30 10])


subplot(6, 2, 5)
plot(-t3.distance_m(ind3)+offset, t3.velocity_mps(ind3),'LineWidth',2,'FontSize',12)
ylim([0,8])
ylabel('Trial 3')
xlim([-30 10])


subplot(6, 2, 6)
plot(-t3.distance_m(ind3)+offset, t3.ddxV(ind3),'LineWidth',2)
hold on;
plot(-t3.distance_m(ind3)+offset, t3.axDS(ind3),'r','LineWidth',2)
xlim([-30 10])


subplot(6, 2, 7)
plot(-t4.distance_m(ind4)+offset, t4.velocity_mps(ind4),'LineWidth',2)
ylim([0,8])
ylabel('Trial 4')
xlim([-30 10])

subplot(6, 2, 8)
plot(-t4.distance_m(ind4)+offset, t4.ddxV(ind4),'LineWidth',2)
hold on;
plot(-t4.distance_m(ind4)+offset, t4.axDS(ind4),'r','LineWidth',2)
xlim([-30 10])


subplot(6, 2, 9)
plot(-t5.distance_m(ind5)+offset, t5.velocity_mps(ind5),'LineWidth',2)
ylabel('Trial 5')
ylim([0,8])
xlim([-30 10])


subplot(6, 2, 10)
plot(-t5.distance_m(ind5)+offset, t5.ddxV(ind5),'LineWidth',2)
hold on;
plot(-t5.distance_m(ind5)+offset, t5.axDS(ind5),'r','LineWidth',2)
xlim([-30 10])

subplot(6, 2, 11)
plot(-t6.distance_m(ind6)+offset, t6.velocity_mps(ind6),'LineWidth',2)
ylim([0,8])
ylabel('Trial 6')
xlabel('Distance to Crosswalk (m)')
xlim([-30 10])


subplot(6, 2, 12)
plot(-t6.distance_m(ind6)+offset, t6.ddxV(ind6),'LineWidth',2)
hold on;
plot(-t6.distance_m(ind6)+offset, t6.axDS(ind6),'r','LineWidth',2)
legend('Desired','Actual')
xlabel('Distance to Crosswalk (m)')
xlim([-30 10])
