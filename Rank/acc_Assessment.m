%停车姿态精度acc：泊车最终位姿与目标位姿的横、纵向偏差和航向角偏差的加权值
% 0-5 5-10 10-15 
% 0-1 1-2 2-3 3+
function [score]=acc_Assessment(xError,yError,HeadingAngleError)
% step
x0 = 0;  % unit: m
x1 = 0.05;
x2 = 0.1;
x3 = 0.15;

y0 = 0;  % unit: m
y1 = 0.05;
y2 = 0.1;
y3 = 0.15;

d0 = 0;
d1 = pi/180;
d2 = pi/120;
d3 = pi/90;

%纵向偏差
if (x0<=xError)&&(xError<=x1)
    score_xError=10;
elseif (x1<=xError)&&(xError<=x2)
    score_xError=6;
elseif (x2<=xError)&&(xError<=x3)
    score_xError=2;
elseif xError<0
    fprintf("xError<0\n")
    score_xError=0;
else
    score_xError=0;
end

%横向偏差
if (y0<=yError)&&(yError<=y1)
    score_yError=10;
elseif (y1<=yError)&&(yError<=y2)
    score_yError=6;
elseif (y2<=yError)&&(yError<=y3)
    score_yError=2;
elseif yError<0
    fprintf("yError<0\n")
    score_yError=0;
else
    score_yError=0;
end

%航向角偏差
if (d0<=HeadingAngleError)&&(HeadingAngleError<=d1)
    score_HeadingAngleError=10;
elseif (d1<=HeadingAngleError)&&(HeadingAngleError<=d2)
    score_HeadingAngleError=6;
elseif (d2<=HeadingAngleError)&&(HeadingAngleError<=d3)
    score_HeadingAngleError=2;
elseif HeadingAngleError<0
    fprintf("HeadingAngelError<0\n")
    score_HeadingAngleError=0;
else
    score_HeadingAngleError=0;
end

score=score_xError*0.3+score_yError*0.3+score_HeadingAngleError*0.4;
end