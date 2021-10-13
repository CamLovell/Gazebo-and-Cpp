clear;
clc;
close all;
rosinit
addpath("~/NUMarine_ws/src/navigation");
rosgenmsg
sub = rossubscriber('/wamv/sensors/lidars/lidar_wamv/points');
pause(1);

msg = receive(sub,10)

rosshutdown
