function [isHighRisk, minDistance, collisionFlag] = Risk_Assessment(refPose, obstaclePose, carPose)
% function [Is_High_Risk, distoobs] = Risk_Assessment(refPose, obstaclePose, carPose)
%**********************************************************************************************
%判断车辆是否存在与前后障碍车碰撞的风险
%与障碍车距离<0.1m时，Is_High_Risk = true
%长度单位统一为m 角度单位统一为rad
%refPose和obstaclePose内坐标排序按点2,1,3的顺序排布(内，角，外)
%carPose内坐标排序按点1~8顺序排布(从左前顺时针环绕，最终回到左后视镜)
% *********************************************************************************************
% 输出量初始化
isHighRisk = 1;
minDistance = NaN;
collisionFlag = 0;
DisFun = @(a,b) cross(a,b)./norm(b);
% 输入量转换成三维坐标
refPose(3, :) = 0;
obstaclePose(3, :) = 0;
carPose(3, :) = 0;

%% 检测碰撞
% 计算车辆角点到障碍物边线(直线)的距离
disCar2Ref = NaN .* ones(3, 16);
disCar2Obs = NaN .* ones(3, 24);

vecRefPose = diff(refPose, 1, 2);  
auxVecRef2Car = carPose - refPose(:,2);
disCar2Ref(:, 1:8) = bsxfun(DisFun, auxVecRef2Car, vecRefPose(:,1));
disCar2Ref(:, 9:16) = bsxfun(DisFun, auxVecRef2Car, vecRefPose(:,2));

vecObstaclePose = diff(obstaclePose, 1, 2);
auxVecObs2Car = carPose - obstaclePose(:,2);
auxVecObs2Car2 = carPose - obstaclePose(:,4);
disCar2Obs(:, 1:8) = bsxfun(DisFun, auxVecObs2Car, vecObstaclePose(:,1));
disCar2Obs(:, 9:16) = bsxfun(DisFun, auxVecObs2Car, vecObstaclePose(:,2));
disCar2Obs(:, 17:24) = bsxfun(DisFun, auxVecObs2Car2, vecObstaclePose(:,3));

% 计算障碍物角点到车子边线(直线)的距离
disRef2Car = NaN .* ones(3, 8);
disObs2Car = NaN .* ones(3, 8);
vecCarPose = diff([carPose carPose(:,1)], 1, 2);

for num = 2 : 2 : 8
    auxVecRef2Car = carPose(:, num) - refPose(:,2);
    disRef2Car(:, num-1 : num) = bsxfun(DisFun, auxVecRef2Car, vecCarPose(:, num-1 : num));
    
    auxVecObs2Car = carPose(:, num) - obstaclePose(:,2);
    disObs2Car(:, num-1 : num) = bsxfun(DisFun, auxVecObs2Car, vecCarPose(:, num-1 : num));
end

% 判断Ref的角点和Obs的角点是否在车辆内部
if sum(disRef2Car(3,:) <= 0) == 8
    collisionFlag = 1;  %和前车发生碰撞
    return;
elseif sum(disObs2Car(3,:) <=0) == 8
    collisionFlag = 2;  %和后车发生碰撞
    return;
else
    for num = 1 : 8
        judgeRef = disCar2Ref(:,[num 8+num]);
        judgeObs = disCar2Obs(:,[num 8+num 16+num]);
        if sum(judgeRef(3,:) >=0) == 2
            collisionFlag = 1;  %和前车发生碰撞
            return;
        elseif sum(judgeObs(3,:) <=0) == 3
            collisionFlag = 2;  %和后车发生碰撞
            return;
        end
    end
end

%% 检测是否超出路沿
transTheta = atan2(refPose(2,2) - obstaclePose(2,2), refPose(1,2) - obstaclePose(1,2));
transMatrix = [cos(transTheta) -sin(transTheta) obstaclePose(1,2); ...
               sin(transTheta) cos(transTheta) obstaclePose(2,2); ...
               0                0               1];
judgeCarPose = [carPose(1:2,:); ones(1,8)];
carPoseInObs = transMatrix \ judgeCarPose;
if any (carPoseInObs(2,:) < -2.05)
    collisionFlag = 3;   %和路沿发生碰撞
    return;
end
%% 计算最小距离
% 计算车辆角点到障碍物边线(线段)的距离
disCar2Ref(1:2,:)=[];
for num2P = 1 : 2
    for num = 1 : 8
        auxVecRef2Car = carPose(:, num) - refPose(:, num2P);
        k = dot(auxVecRef2Car, vecRefPose(:, num2P)) / norm(vecRefPose(:, num2P))^2;
        k1 = k > 1;
        k2 = k < 0;
        k3 = ~(k1||k2);
        numSave = num + 8*(num2P-1);
        disCar2Ref(k1, numSave) = norm(carPose(:, num) - refPose(:, num2P + 1));
        disCar2Ref(k2, numSave) = norm(auxVecRef2Car);
        disCar2Ref(k3, numSave) = abs(disCar2Ref(numSave));
    end
end

disCar2Obs(1:2,:)=[];
for num2P = 1 : 3
    for num = 1 : 8
        auxVecObs2Car = carPose(:, num) - obstaclePose(:, num2P);
        k = dot(auxVecObs2Car, vecObstaclePose(:, num2P)) / norm(vecObstaclePose(:, num2P))^2;
        k1 = k > 1;
        k2 = k < 0;
        k3 = ~(k1||k2);
        numSave = num + 8*(num2P-1);
        disCar2Obs(k1, numSave) = norm(carPose(:, num) - obstaclePose(:, num2P + 1));
        disCar2Obs(k2, numSave) = norm(auxVecObs2Car);
        disCar2Obs(k3, numSave) = abs(disCar2Obs(numSave));
    end
end
% 计算障碍物角点到车子边线(线段)的距离
carPose = [carPose, carPose(:,1)];
disRef2Car = NaN .* ones(1, 24);
for num8P = 1 : 8
    for num = 1 : 3
        auxVecCar2Ref = refPose(:, num) - carPose(:, num8P);
        k = dot(auxVecCar2Ref, vecCarPose(:, num8P)) / norm(vecCarPose(:, num8P))^2;
        k1 = k > 1;
        k2 = k < 0;
        k3 = ~(k1||k2);
        numSave = num + 3*(num8P-1);
        disRef2Car(k1, numSave) = norm(refPose(:, num) - carPose(:, num8P + 1));
        disRef2Car(k2, numSave) = norm(auxVecCar2Ref);
        if k3
            tmpDis = DisFun(auxVecCar2Ref, vecCarPose(:, num8P));
            disRef2Car(k3, numSave) = abs(tmpDis(3));
        end
    end
end
disObs2Car = NaN .* ones(1, 24);
for num8P = 1 : 8
    for num = 1 : 3
        auxVecCar2Obs = obstaclePose(:, num) - carPose(:, num8P);
        k = dot(auxVecCar2Obs, vecCarPose(:, num8P)) / norm(vecCarPose(:, num8P))^2;
        k1 = k > 1;
        k2 = k < 0;
        k3 = ~(k1||k2);
        numSave = num + 3*(num8P-1);
        disObs2Car(k1, numSave) = norm(obstaclePose(:, num) - carPose(:, num8P + 1));
        disObs2Car(k2, numSave) = norm(auxVecCar2Obs);
        if k3
            tmpDis = DisFun(auxVecCar2Obs, vecCarPose(:, num8P));
            disObs2Car(k3, numSave) = abs(tmpDis(3));
        end
    end
end

% 选出最小距离
minDistance = min([disCar2Obs, disCar2Ref, ...
    disRef2Car, disObs2Car]);

if minDistance > 0.1
    isHighRisk = 0;
end
end