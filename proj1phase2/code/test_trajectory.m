%  Used for HKUST ELEC 6910P 

close all;
clear all;
clc;
addpath('./utils','./readonly');

path1 = [0.0 0.0 1.0 ; ...
         1.0 1.0 1.0 ; ...
         -1.0 2.0 1.0 ; ...
         1.0 3.0 1.0 ; ...
         -1.0 4.0 1.0 ; ...
         1.0 5.0 1.0 ; ...
         -1.0 6.0 1.0 ; ...
         1.0 7.0 1.0 ; ...
         -1.0 8.0 1.0 ; ...
         1.0 9.0 1.0 ; ...
         0.0 10.0 1.0 ; ];
     
path2 = [0.5 0.5 1.0 ; ...
        2.0 0.5 1.0 ; ...
        2.0 2.0 1.0 ; ...
        0.5 2.0 1.0 ; ...
        0.5 0.5 1.0 ; ...
        2.0 0.5 1.0 ; ...
        2.0 2.0 1.0 ; ...
        0.5 2.0 1.0 ; ...
        0.5 0.5 1.0 ];
    
path3 = [0.0 0.0 0.0;...
         0.0 1.0 1.0;...
        -1.0 0.0 2.0;...
         0.0 -1.0 3.0;...
         1.0 0.0 4.0;...
         0.0 1.0 5.0;...
        -1.0 0.0 6.0;...
         0.0 -1.0 7.0;...
         1.0 0.0 8.0];
     
path4 = [0.0 1.0 2.0;...
         1.0 2.0 3.0;...
         0.0 1.0 2.0;...
         1.0 2.0 3.0;...
         0.0 1.0 2.0;...
         1.0 2.0 3.0;];


h1 = subplot(3,3,1);
h2 = subplot(3,3,2);
h3 = subplot(3,3,3);
h4 = subplot(3,3,4);
h5 = subplot(3,3,5);
h6 = subplot(3,3,6);
h7 = subplot(3,3,7);
h8 = subplot(3,3,8);
h9 = subplot(3,3,9);
set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [100, 100, 1400, 1000]);
% set(gcf, 'WindowStyle','Modal');


% Trajectory Generator
trajectory_generator([], path2, h1);

% Run Trajectory
run_trajectory_readonly(h1, h2, h3, h4, h5, h6, h7, h8, h9);
