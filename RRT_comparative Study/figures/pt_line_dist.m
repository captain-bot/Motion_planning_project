clc
clear
close all

pt = [0.5; 0.65];
line_seg = [0, 0; 1, 1];

w = [line_seg(1, 2) - line_seg(2, 2);
    line_seg(2, 1) - line_seg(1, 1)];
b =  (line_seg(2, 2) - line_seg(1, 2))*line_seg(1, 1) ...
       - (line_seg(2, 1) - line_seg(1, 1))*line_seg(1, 2);

dist = w'*pt + b;
