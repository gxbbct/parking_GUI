%入位总时长T，姿态精度acc，碰撞风险crash，舒适性com，原地转向时长rot
%泊车入位总时长T：从开始泊车到泊车结束的总时间
%停车姿态精度acc：泊车最终位姿与目标位姿的横0.3、纵向偏差0.3和航向角偏差0.4的加权值
%碰撞风险risk：泊车过程中车辆与障碍车距离小于10cm的总时长
%舒适性com：泊车过程中横向加速度大于3.5m/s^2或纵向加速度大于3.5m/s^2的总时长(阈值可根据情况自己定义)
%原地转向时长rot：泊车过程中驾驶员原地打方向盘的总时长
% function [score]=eva(Time,xError,yError,HeadingAngelError,RefPose1,RefPose2,ObstaclePose1,ObstaclePose2,Pos_Car,Vehicle,LocalAx,LocalAy,VehicleSpeed,angle)

%Time_score=T_Assessment(Time);
%acc_score=acc_Assessment(xError,yError,HeadingAngelError);
%risk_score=RiskAssess(RefPose1,RefPose2,ObstaclePose1,ObstaclePose2,Pos_Car,Vehicle);
%com_score=com_Assessment(LocalAx,LocalAy);
%rot_score=rot_Assessment(VehicleSpeed,angle);

function [score_return]=eva(Time_score,acc_score,risk_score,com_score,rot_score)
% A0=[1, 3, 4, 7,	9;...
%         1/3, 1, 2, 5, 7;...
%         1/4, 1/2, 1, 4, 6;...
%         1/7, 1/5, 1/4, 1, 3;...
%         1/9, 1/7, 1/6, 1/3, 1];
% [v,d]=eigs(A0);
% lamdamax=max(d(:));
% [m,n]=size(v); 
% %将特征向量标准化  
% sum = 0;  
% for i=1:m  
%     sum = sum + v(i,1);  
% end  
% w0 = v(:,1);  
% for i=1:m  
%     w0(i,1)= v(i,1)/sum;  
% end  
% 
% CI=(lamdamax-5)/4;

w0 = [0.4993; 0.2436; 0.1631; 0.0613; 0.0327];
score = w0 .* [Time_score; acc_score; risk_score; com_score; rot_score];
percent = score ./ 10;  % percentage in total
ppercent = percent ./ w0;  % percentage in their own element

score_return = [w0, score, percent, ppercent];

% score=Time_score*w0(1,1)+acc_score*w0(2,1)+risk_score*w0(3,1)+com_score*w0(4,1)+rot_score*w0(5,1);

end



