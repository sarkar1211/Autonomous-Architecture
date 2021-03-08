clear all
clc
close all

F = 10;
m = 5;
c = 3;
b = 2;

sim('test')

figure
plot(time,distance);
xlabel('Time [s]')
ylabel('Distance [m]')
title('Distance Plot')

figure
plot(time,velocity);
xlabel('Time [s]')
ylabel('Velocity [m/s]')
title('Velocity Plot')

figure
plot(time,acceleration);
xlabel('Time [s]')
ylabel('Acceleration [m/s^2]')
title('Acceleration Plot')