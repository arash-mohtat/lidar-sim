function [firingSequenceCycle,dt,ID_vertAngl_vertCorr]=VLP16parameters()

firingSequenceCycle=[0:15,NaN*ones(1,8)]; % firing order of laser IDs (one full cycle)
dt=2.304e-6;   % length between firing events in sec
ID_vertAngl_vertCorr = [ ...
    0 ,-15,0.0112
    1 , 1 ,-0.0007
    2 ,-13,0.0097
    3 , 3 ,-0.0022
    4 ,-11,0.0081
    5 , 5 ,-0.0037
    6 , -9,0.0066
    7 , 7 ,-0.0051
    8 , -7,0.0051
    9 , 9 ,-0.0066
    10, -5,0.0037
    11, 11,-0.0081
    12, -3,0.0022
    13, 13,-0.0097
    14, -1,0.0007
    15, 15,-0.0112 ];