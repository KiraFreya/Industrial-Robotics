function Lab1_Exercise
%% Clear & close
close all;
clc
%% 
% Picture
imshow('Lab1CircularRaceTrack.jpg');
axis on
hold on
%% Transform Car 1
T1 = SE2(300,550,0);
T1_h = trplot2(T1, 'frame', '1', 'color', 'b','length',50);
%% Transform 2
T2 = SE2(300, 125, 0);
hold on
T2_h = trplot2(T2, 'frame', '2', 'color', 'r');

end