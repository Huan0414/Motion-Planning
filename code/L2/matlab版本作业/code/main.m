% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; 
% clc;
addpath('A_star')

% Environment map in 2D space 
xStart = 1.0;
yStart = 1.0;
xTarget = 250.0;
yTarget = 250.0;
MAX_X = 250;
MAX_Y = 250;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);

hn_option = 'Distance'; % (Euclidean)'Distance', 'Manhattan', or 'Dijkstra'

tic
% Waypoint Generator Using the A* 
[path, OPEN] = A_star_search(map, MAX_X,MAX_Y, hn_option);
toc

% visualize the 2D grid map
visualize_map(map, path, OPEN);

% save map
% save('Data/map.mat', 'map', 'MAX_X', 'MAX_Y');
