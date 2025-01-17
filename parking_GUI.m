function varargout = parking_GUI(varargin)
% PARKING_GUI MATLAB code for parking_GUI.fig 
%      PARKING_GUI, by itself, creates a new PARKING_GUI or raises the existing
%      singleton*.
%
%      H = PARKING_GUI returns the handle to a new PARKING_GUI or the handle to
%      the existing singleton*.
%
%      PARKING_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PARKING_GUI.M with the given input arguments.
%
%      PARKING_GUI('Property','Value',...) creates a new PARKING_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before parking_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to parking_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help parking_GUI

% Last Modified by GUIDE v2.5 04-Oct-2021 20:46:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @parking_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @parking_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before parking_GUI is made visible.
function parking_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to parking_GUI (see VARARGIN)

% addpath('Callbacks');
addpath('Rank');
global PubArrayText % show interaction message
global flag_show    % related to trajectory showing
global flag_pk      % related to the ui.pk visibility
global h_tr         % handles of trajectory
global machine      % is machine?
flag_show = 0;
flag_pk = 0;
h_tr = 0;
machine = 1;

rosshutdown;
rosinit;
% InitTrajectory(handles);

PubArrayText = sprintf('%s\n%s\n','点击“开始泊车”进行泊车数据显示','点击“退出”关闭界面');
set(handles.Notice,'String',PubArrayText);

% Choose default command line output for parking_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

set(handles.ui_paraset, 'Visible', 1);
set([handles.ui_show, handles.ui_tr, handles.ui_msg, handles.ui_score], 'Visible', 0);



% UIWAIT makes parking_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = parking_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
global stop;   % stop parking
global exit;  % close GUI
global tr_xlim;  % xlim of handles.Trajectory
global tr_ylim;  % ylim of handles.Trajectory
global p_width; % width of target position
global vehiclePose;
global last_Ref;
global xError;
global yError;
global HeadingAngleError;
global risk_score;
global init_flag;
global machine;

risk_score = 0;
stop = 1;   
exit = 0;  
last_Ref = zeros(2,20);
init_flag = 0;
parkingtime = 0; %泊车时间
minDis = 100;  %最小安全距离

%整车参数
vehicle_width = 1.551; %车宽
vehicle_length = 3.569; %车长
rear_overhang = 0.544; %后悬
front_overhang = 0.72; %前悬
wheel_base = 2.305;%轴距
front_wheel_track = 1.324;%前轮距
rear_wheel_track = 1.292;%后轮距
front_vehicle_width = 0.791; %对应八边形f边
rear_vehicle_width = 0.821; %对应八边形c边

Vehicle.Wf = front_vehicle_width;
Vehicle.Wr = rear_vehicle_width;
Vehicle.Lf = front_overhang + wheel_base;
Vehicle.Lr = rear_overhang;

%% rossubscriber initialization
angle = myvector(2);
angle_indice = 0;
sub_steering_angle = rossubscriber('/steering_angle_deg', 'apa_msgs/SteeringAngleStamped',{@SteeringAngleCallback, angle}, 'BufferSize', 5);
angleSub = rossubscriber('/steering_angle_deg', 'apa_msgs/SteeringAngleStamped');

VehicleSpeed = myvector(2);
VehicleSpeed_indice = 0;
velometer = rossubscriber('/velometer/base_link_local', 'geometry_msgs/TwistStamped',{@velometerCallback,VehicleSpeed});
vehicleSpeedSub = rossubscriber('/velometer/base_link_local', 'geometry_msgs/TwistStamped');

LocalA = myvector(3);
LocalA_indice = 0;
imu = rossubscriber('/imu/data', 'sensor_msgs/Imu',{@imuCallback,LocalA});
localASub = rossubscriber('/imu/data', 'sensor_msgs/Imu');

%       {time,Ref1,Ref2,Ref3,Ref4,Obstacle1,Obstacle2,Obstacle3,Obstacle4,refposetheta}
parkingSlot = myvector(18);
parkingSlot_indice = 0;
parking_slot = rossubscriber('/parking_slot_info', 'apa_msgs/SlotInfoStamped',{@parkingslotCallback,parkingSlot}, 'BufferSize', 1);
parkSub = rossubscriber('/parking_slot_info', 'apa_msgs/SlotInfoStamped');

%       {time,localx,localy,yaw}
vehiclePose = myvector(4);
vehiclePose_indice = 0;
Vehicle_pose2D = rossubscriber('/odometer/local_map/base_link', 'nav_msgs/Odometry',{@Vehicle_pose2DCallback,vehiclePose}, 'BufferSize', 1);
vehicleSub = rossubscriber('/odometer/local_map/base_link', 'nav_msgs/Odometry');

% P-8, R-7, N-6, D-5
gearSub = rossubscriber('/shift_position', 'apa_msgs/IntStamped');

% start-1, stop-0;
enableSub = rossubscriber('/time_enable', 'apa_msgs/IntStamped');

pause(1);
setlog(handles, '话题订阅成功！');
myCount = 0;
mytimer = tic;

%% main loop
while(~exit)
%     tic
    try
        if (gearSub.LatestMessage.Value ~= 8 && stop == 1 && ~machine) || ...
                (gearSub.LatestMessage.Value == 8 && stop == 0 && ~machine) || ...
                (enableSub.LatestMessage.Value == 1 && stop == 1 && machine) || ...
                (enableSub.LatestMessage.Value == 0 && stop == 0 && machine)
            StartStop(handles);
        end
    catch ME         
    end
        
    pause(0.05);
    %% when the parking process starts
    if ~(stop||init_flag)
        %创建订阅器与选择显示模块
        angle.clear();
        angle_indice = 0;
        
        VehicleSpeed.clear();
        VehicleSpeed_indice = 0;
        
        LocalA.clear();
        LocalA_indice = 0;

%       {time,Ref1,Ref2,Ref3,Ref4,Obstacle1,Obstacle2,Obstacle3,Obstacle4,refposetheta}
        parkingSlot.clear();
        parkingSlot_indice = 0;
        
%       {time,localx,localy,yaw}
        vehiclePose.clear();
        vehiclePose_indice = 0;
        
        setlog(handles, '容器已重置。');

        %Display modules initialization
        clear_graph(handles);
        
        set(handles.Time, 'String', '');
        set(handles.score, 'String', '');
        
        %Timer initialization
        set(handles.timer, 'Visible', 1);
        
        init_flag = 1;  % finish initialization
        mytimer = tic;
        myCount = 0;
    end
    
    %% when the parking process finishes
    if stop&&init_flag
        recordnum = [size(angle), size(VehicleSpeed), size(LocalA), size(parkingSlot), size(vehiclePose)];
        if recordnum(4)~=0
            % record the latest position
            last_Ref = [p_fl(1:2) p_fr(1:2) p_rr(1:2) p_rl(1:2) ...
                Ref_fl Ref_fr(1:2) Ref_rr(1:2) Ref_rl ...
                Obs_fl Obs_fr(1:2) Obs_rr(1:2) Obs_rl ...
                V1G(1:2) V2G(1:2) V3G(1:2) V4G(1:2) ...
                V5G(1:2) V6G(1:2) V7G(1:2) V8G(1:2)];
            
            % judge: did the car drive into the parking lot? (1-no, 0-yes)
            standardLine = Ref_rl - Obs_fl;
            standardLine(3) = 0;
            lineGroup = last_Ref(:,13:18) - Obs_fl;
            lineGroup(3,:) = 0;
            judgeMatrix = zeros(3, 6);
            for number = 1 : 6
                judgeMatrix(:, number) = cross(standardLine, lineGroup(:,number)) > 0;
            end
            if sum(judgeMatrix(3,:)) == 0
                %泊车时间评分
                Time_score = T_Assessment(parkingtime);
                fprintf('Time_score =%d\n', Time_score);
                
                %姿态精度评分
                angleError = deg2rad(HeadingAngleError);
                acc_score = acc_Assessment(abs(xError),abs(yError),abs(angleError));
                fprintf('acc_score =%d\n', acc_score);
                
                %舒适度评分
                Acc = get_above(LocalA, recordnum(3));
                %             LocalAx = ;   %纵向加速度
                %             LocalAy = ;   %横向加速度
                com_score = com_Assessment(Acc(:,2),Acc(:,3));
                fprintf('com_score =%d\n', com_score);
                
                %原地转向时长评分
                %             VehicleSpeed_data = ;
                %             angle_data = ;
                rot_score = rot_Assessment(get_above(VehicleSpeed, recordnum(2)),get_above(angle, recordnum(1)));
                fprintf('rot_score =%d\n', rot_score);
                
                %计算碰撞风险评分
                if isnan(minDis)
                    risk = 0;
                else
                    set(handles.distance, 'String', ['本次泊车过程中与障碍物的最小距离为：' num2str(minDis,'%.2f') 'm'], 'Fontsize', 13, 'ForegroundColor', 'r');
                    risk=Risk(risk_score, recordnum(4));
                end
                fprintf('risk_score =%d\n', risk);
                % score = [weight of every element; score of every element; percent of every element]
                score = eva(Time_score,acc_score,risk,com_score,rot_score);
                
                set(handles.timeScore, 'String', [num2str(score(1,2), '%.2f'), '/', num2str(10*score(1,1),'%.2f'), '            ', num2str(round(100 * score(1,4)), '%d'), '%']);
                set(handles.accScore, 'String', [num2str(score(2,2), '%.2f'), '/', num2str(10*score(2,1),'%.2f'), '            ', num2str(round(100 * score(2,4)), '%d'), '%']);
                set(handles.safetyScore, 'String', [num2str(score(3,2), '%.2f'), '/', num2str(10*score(3,1),'%.2f'), '            ', num2str(round(100 * score(3,4)), '%d'), '%']);
                set(handles.comScore, 'String', [num2str(score(4,2), '%.2f'), '/', num2str(10*score(4,1),'%.2f'), '            ', num2str(round(100 * score(4,4)), '%d'), '%']);
                set(handles.rotScore, 'String', [num2str(score(5,2), '%.2f'), '/', num2str(10*score(5,1),'%.2f'), '            ', num2str(round(100 * score(5,4)), '%d'), '%']);
    
                try
                    scoreList = readtable(['DataSave/成绩记录' datestr(now, 'yymmdd')]);
                catch ME
                    if all(ME.identifier == 'MATLAB:textio:textio:FileNotFound')
                        set(handles.pkResult, 'String', '今日暂无机器成绩记录', 'FontSize', 18, 'ForegroundColor', 'k');
                    else
                        set(handles.pkResult, 'String', ME.identifier, 'FontSize', 14, 'ForegroundColor', 'k');
                    end
                end
                
                if exist('scoreList', 'var') && ~handles.St_Robot.Value
                    scoreBot = scoreList(ismissing(scoreList(:,2), '机器'),:);
                    if ~isempty(scoreBot)
                        botScoreArray = table2array(scoreBot(end, 4 : 9))';
                        humanScoreArray = [sum(score(:,2)); score(:,2)];
                        redArray = [handles.myScore, handles.myTimeScore, handles.myAccScore, ...
                            handles.mySafetyScore, handles.myComScore, handles.myRotScore];
                        set(redArray, {'String'}, cellfun(@(x) num2str(x, '%.2f'), ...
                            num2cell(humanScoreArray), 'UniformOutput', false));
                        greenArray = redArray;
                        redArray(humanScoreArray <= botScoreArray) = [];
                        greenArray(humanScoreArray >= botScoreArray) = [];
                        set(redArray, 'BackgroundColor', 'red');
                        set(greenArray, 'BackgroundColor', 'green');
                        set([handles.botScore, handles.botTimeScore, handles.botAccScore, ...
                            handles.botSafetyScore, handles.botComScore, handles.botRotScore], {'String'}, ...
                            cellfun(@(x) num2str(x, '%.2f'), num2cell(botScoreArray), 'UniformOutput', false));
                        if humanScoreArray(1) > botScoreArray(1)
                            set(handles.pkResult, 'String', '胜利', 'ForegroundColor', 'red','FontSize', 50);
                        elseif humanScoreArray(1) == botScoreArray(1)
                            set(handles.pkResult, 'String', '平局', 'ForegroundColor', 'black', 'FontSize', 50);
                        else
                            set(handles.pkResult, 'String', '失败', 'ForegroundColor', 'black', 'FontSize', 50);
                        end
                        set(handles.push_pk, 'Visible', 1);
                    end
                end
                
                set(handles.ui_ScoreDetail, 'Visible', 1);
            else
                score = zeros(5,4);
                setlog(handles, '车辆未完全泊入库位中，本次泊车成绩为0分。');
            end
            set(handles.Time,'string',[num2str(parkingtime,'%.2f'), ' s']);
            set(handles.score,'string',num2str(sum(score(:,2)),'%.2f'));
            
            % saving data
            if get(handles.St_save,'Value')
                % saving messages
                if ~exist('DataSave', 'dir')
                    mkdir('DataSave');
                end
                saveFileName = ['DataSave/' get(handles.set_name,'String') '_' datestr(now,'yymmddHHMM') '.xls'];
                setlog(handles, '正在保存数据...');
                writematrix(angle.get_above(recordnum(1)), saveFileName, 'Sheet', '!steering_angle_deg');
                writematrix(VehicleSpeed.get_above(recordnum(2)), saveFileName, 'Sheet', '!velometer!base_link_local');
                writematrix(LocalA.get_above(recordnum(3)), saveFileName, 'Sheet', '!imu!data');
                writematrix(parkingSlot.get_above(recordnum(4)), saveFileName, 'Sheet', '!parking_slot_info');
                writematrix(vehiclePose.get_above(recordnum(5)), saveFileName, 'Sheet', '!odometer!local_map!base_link');
                
                % saving score
                saveFileName = ['DataSave/成绩记录' datestr(now, 'yymmdd') '.xls'];
                if ~exist(saveFileName, 'file')
                    writetable(table({'实验时间'},{'姓名'},{'泊车时间'},...
                        {'泊车总分'},{'泊车时间分'},{'精度分'},{'安全分'},...
                        {'舒适度分'},{'原地转向分'}), saveFileName, 'WriteVariableNames',false);
                end
                writetable(table({datestr(now, 'yymmdd-HH-MM')}, ...
                    {get(handles.set_name, 'String'), parkingtime, ...
                    sum(score(:, 2)), score(1,2), score(2,2), score(3,2), ...
                    score(4,2) score(5,2)}), saveFileName, 'WriteMode','Append', ...
                    'WriteVariableNames',false);
                
                setlog(handles, '保存完毕，请继续。');
            end
            % show the button
            set(handles.push_show,'Visible',1);
        end    
        init_flag = 0;
        set(handles.timer, 'Visible', 0);
    end
          
    %%  Get and display message
    if exit
        break;
    end
    
%     [latest_angle_record, angle_indice, ~] = getmsg(angle, angle_indice, handles);
    latest_angle_record = angleSub.LatestMessage;
    if isempty(latest_angle_record)
        latest_angle_record = NaN;
    else
        latest_angle_record = latest_angle_record.Angle;
    end
    pause(0.02);
    
%     [latest_A_record, LocalA_indice, ~] = getmsg(LocalA, LocalA_indice, handles);
    latest_A_record = localASub.LatestMessage;
    if isempty(latest_A_record)
        latest_A_record = [NaN; NaN];
    else
        latest_A_record = [latest_A_record.LinearAcceleration.X; latest_A_record.LinearAcceleration.Y];
    end
    pause(0.02);
    
    
%     [latest_speed_record, VehicleSpeed_indice, ~] = getmsg(VehicleSpeed, VehicleSpeed_indice, handles);    
    latest_speed_record = vehicleSpeedSub.LatestMessage;
    if isempty(latest_speed_record)
        latest_speed_record = NaN;
    else
        latest_speed_record = sqrt(latest_speed_record.Twist.Linear.X^2 + latest_speed_record.Twist.Linear.Y^2);
    end
    pause(0.02);
    
%     [latest_vehiclePose_record, vehiclePose_indice, updateVehiclePose] = getmsg(vehiclePose, vehiclePose_indice, handles);
%     LocalX = latest_vehiclePose_record(2); LocalY = latest_vehiclePose_record(3);
%     Yaw = latest_vehiclePose_record(4);
    

    latest_vehiclePose_record = vehicleSub.LatestMessage;
    if isempty(latest_vehiclePose_record)
        LocalX = NaN;
    else
        fprintf([num2str(latest_vehiclePose_record.Header.Stamp.seconds, '%.2f') '\n']);
        LocalX = latest_vehiclePose_record.Pose.Pose.Position.X;
        LocalY = latest_vehiclePose_record.Pose.Pose.Position.Y;
        q = [latest_vehiclePose_record.Pose.Pose.Orientation.W latest_vehiclePose_record.Pose.Pose.Orientation.X ...
            latest_vehiclePose_record.Pose.Pose.Orientation.Y latest_vehiclePose_record.Pose.Pose.Orientation.Z];
        EulAngle = quat2eul(q);
        Yaw = EulAngle(1);
    end

    
%     [latest_parkingSlot_record, parkingSlot_indice, updateParkingSlot] = getmsg(parkingSlot, parkingSlot_indice, handles);
%     RefPose1 = latest_parkingSlot_record(2:3)'; RefPose2 = latest_parkingSlot_record(4:5)';
%     RefPose3 = latest_parkingSlot_record(6:7)'; RefPose4 = latest_parkingSlot_record(8:9)';
%     ObstaclePose1 = latest_parkingSlot_record(10:11)'; ObstaclePose2 = latest_parkingSlot_record(12:13)';
%     ObstaclePose3 = latest_parkingSlot_record(14:15)'; ObstaclePose4 = latest_parkingSlot_record(16:17)';
%     RefPoseTheta = latest_parkingSlot_record(18);

      latest_parkingSlot_record = parkSub.LatestMessage;
    if isempty(latest_parkingSlot_record)
        RefPose1 = NaN;
    else
        RefPoseX = latest_parkingSlot_record.RefPose.X;
        RefPoseY = latest_parkingSlot_record.RefPose.Y;
        RefPoseTheta = latest_parkingSlot_record.RefPose.Theta;
        RefExtend_X = latest_parkingSlot_record.RefExtendX;
        RefExtend_Y = latest_parkingSlot_record.RefExtendY;
        
        T = [cos(RefPoseTheta), -sin(RefPoseTheta), RefPoseX; sin(RefPoseTheta), cos(RefPoseTheta), RefPoseY; 0, 0, 1];
        RefPose1 = [RefPoseX;RefPoseY];
        b = [ 0; RefExtend_Y; 1];
        c = [ RefExtend_X;0;1];
%         d = [ RefExtend_X;RefExtend_Y; 1];
        RefPose2 = T*b;  RefPose2(3) = [];
        RefPose3 = T*c;  RefPose3(3) = [];
%         RefPose4 = T*d;  RefPose4(3) = [];
        refPose = [RefPose2, RefPose1, RefPose3];
        
        ObstaclePoseX = latest_parkingSlot_record.ObstaclePose.X;
        ObstaclePoseY = latest_parkingSlot_record.ObstaclePose.Y;
        ObstacleTheta = latest_parkingSlot_record.ObstaclePose.Theta;
        ObstacleExtend_X = latest_parkingSlot_record.ObstacleExtendX;
        ObstacleExtend_Y = latest_parkingSlot_record.ObstacleExtendY;
        T = [cos(ObstacleTheta), -sin(ObstacleTheta), ObstaclePoseX; sin(ObstacleTheta), cos(ObstacleTheta), ObstaclePoseY; 0, 0, 1];
        ObstaclePose1 = [ObstaclePoseX;ObstaclePoseY];
        b = [ 0; ObstacleExtend_Y; 1];
        c = [ ObstacleExtend_X;0;1];
%         d = [ ObstacleExtend_X;ObstacleExtend_Y; 1];
        ObstaclePose2 = T*b;  ObstaclePose2(3) = [];
        ObstaclePose3 = T*c;  ObstaclePose3(3) = [];
%         ObstaclePose4 = T*d;  ObstaclePose4(3) = [];
        obstaclePose = [ObstaclePose2, ObstaclePose1, ObstaclePose3];
    end
    
    
    if ~get(handles.ui_paraset, 'Visible')
        % parking time
        parkingtime = toc(mytimer);
        set(handles.timer, 'String', [num2str(parkingtime, '%.2f'), ' s']);
        %方向盘转角
        set(handles.angle,'string',num2str(latest_angle_record,'%.2f'));
        %车速
        set(handles.VehicleSpeed,'string',num2str(latest_speed_record*3.6,'%.2f'));
        %横向加速度
        set(handles.Localay,'string',num2str(latest_A_record(2)/9.8,'%.2f'));
        %纵向加速度
        set(handles.Localax,'string',num2str(latest_A_record(1)/9.8,'%.2f'));
        
        if ~isnan(RefPose1(1))
            % Target parking position
            VecObs = [ObstaclePose2(1) - ObstaclePose1(1), ObstaclePose2(2) - ObstaclePose1(2)];
            Theta_Obs = atan2(VecObs(2), VecObs(1));
            T_Obs = [cos(Theta_Obs), -sin(Theta_Obs), ObstaclePose1(1); ...
                sin(Theta_Obs), cos(Theta_Obs), ObstaclePose1(2); 0, 0, 1];
            Obs_fr = T_Obs * [p_width; 0; 1];
            Obs_rr = T_Obs * [p_width; -p_width; 1];
            Obs_fl = ObstaclePose1;
            Obs_rl = ObstaclePose3;
            VecRef = [RefPose2(1) - RefPose1(1), RefPose2(2) - RefPose1(2)];
            Theta_Ref = atan2(VecRef(2), VecRef(1));
            T_Ref = [cos(Theta_Ref), -sin(Theta_Ref), RefPose1(1); ...
                sin(Theta_Ref), cos(Theta_Ref), RefPose1(2); 0, 0, 1];
            Ref_rr = T_Ref * [p_width; 0; 1];
            Ref_fr = T_Ref * [p_width; p_width; 1];
            Ref_rl = RefPose1;
            Ref_fl = RefPose3;
            [p_fl, p_fr, p_rr, p_rl, p_length] = calpp(Ref_rl, Ref_rr, Obs_fl, Obs_fr, p_width);
            set(handles.P_length,'string',num2str(p_length,'%.2f'));
            PoseTheta = atan2(p_fl(2) - p_rl(2), p_fl(1) - p_rl(1));
            
            if myCount >= 0
                %绘制算法目标车位  3
                %             color = 'red';
                %             linewidth = 0.5;
                %             linestyle = '-';
                %             PlotPolygon(handles, 3, p_fl, p_fr, p_rr, p_rl);
                set(handles.Trajectory.UserData(3), 'XData', [p_fl(1) p_fr(1) NaN ...
                    p_fr(1) p_rr(1) NaN p_rr(1) p_rl(1) NaN p_rl(1) p_fl(1)], ...
                    'Ydata', [p_fl(2) p_fr(2) NaN p_fr(2) p_rr(2) NaN p_rr(2) p_rl(2) NaN ...
                    p_rl(2) p_fl(2)]);
                
                %             plot([p_fl(1), p_fr(1)],[p_fl(2), p_fr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                %             hold on
                %             plot([p_fr(1), p_rr(1)],[p_fr(2), p_rr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                %             plot([p_rr(1), p_rl(1)],[p_rr(2), p_rl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                %             plot([p_rl(1), p_fl(1)],[p_rl(2), p_fl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                
                %绘制实际泊车空间  4
                %             color = 'green';
                %             linewidth = 0.5;
                %             linestyle = '-';
                %             PlotPolygon(handles, 4, Ref_rl, Ref_rr, Obs_fr, Obs_fl);
                set(handles.Trajectory.UserData(4), 'XData', [Ref_rl(1) Ref_rr(1) NaN ...
                    Ref_rr(1) Obs_fr(1) NaN Obs_fr(1) Obs_fl(1) NaN Obs_fl(1) Ref_rl(1)], ...
                    'Ydata', [Ref_rl(2) Ref_rr(2) NaN Ref_rr(2) Obs_fr(2) NaN Obs_fr(2) Obs_fl(2) NaN ...
                    Obs_fl(2) Ref_rl(2)]);
                %             plot([Ref_rl(1), Ref_rr(1)],[Ref_rl(2), Ref_rr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                %             plot([Ref_rr(1), Obs_fr(1)],[Ref_rr(2), Obs_fr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                %             plot([Obs_fr(1), Obs_fl(1)],[Obs_fr(2), Obs_fl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                %             plot([Obs_fl(1), Ref_rl(1)],[Obs_fl(2), Ref_rl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                
                %绘制障碍物方块  5, 6
                %             color = 'black';
                %             linewidth = 0.5;
                %             linestyle = '-';
                %             PlotPolygon(handles, 5, Obs_fl, Obs_rl, Obs_rr, Obs_fr);
                %             PlotPolygon(handles, 6, Ref_rl, Ref_fl, Ref_fr, Ref_rr);
                
                %             plot([Obs_fl(1),Obs_rl(1)],[Obs_fl(2),Obs_rl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                %             plot([Obs_rl(1),Obs_rr(1)],[Obs_rl(2),Obs_rr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                %             plot([Obs_rr(1),Obs_fr(1)],[Obs_rr(2),Obs_fr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                %             plot([Ref_rl(1),Ref_fl(1)],[Ref_rl(2),Ref_fl(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                %             plot([Ref_fl(1),Ref_fr(1)],[Ref_fl(2),Ref_fr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                %             plot([Ref_fr(1),Ref_rr(1)],[Ref_fr(2),Ref_rr(2)],'Color',color,'LineWidth',linewidth,'LineStyle',linestyle);
                
                %填充  1, 2
                %             color = 'black';
                %             linewidth = 0.5;
                %             linestyle = '--';
                %             facealpha = 0.5;
                
                %             fill([Obs_fl(1), Obs_fr(1), Obs_rr(1), Obs_rl(1)],[Obs_fl(2), Obs_fr(2), Obs_rr(2), Obs_rl(2)],...
                %                 color,'FaceAlpha',facealpha);
                %             fill([Ref_fl(1), Ref_fr(1), Ref_rr(1), Ref_rl(1)],[Ref_fl(2), Ref_fr(2), Ref_rr(2), Ref_rl(2)],...
                %                 color,'FaceAlpha',facealpha);
                %             set(handles.Trajectory,'XLim',tr_xlim,'YLim',tr_ylim);
                set(handles.Trajectory.UserData(1), 'XData', [Obs_fl(1), Obs_fr(1), Obs_rr(1), Obs_rl(1)], ...
                    'YData', [Obs_fl(2), Obs_fr(2), Obs_rr(2), Obs_rl(2)]);
                set(handles.Trajectory.UserData(2), 'XData', [Ref_fl(1), Ref_fr(1), Ref_rr(1), Ref_rl(1)], ...
                    'YData', [Ref_fl(2), Ref_fr(2), Ref_rr(2), Ref_rl(2)]);
                
                myCount = ~myCount;
            end
        end
        
        if ~isnan(LocalX)
            %% 记录当前车辆位置
            %求从车身坐标系到全局坐标系的刚体变换矩阵
            T = [cos(Yaw), -sin(Yaw), LocalX; sin(Yaw), cos(Yaw), LocalY; 0, 0, 1];
            %求车辆八角点在车身坐标系下的位置
            V1L = [3.026;0.3955;1];V2L=[3.026;-0.3955;1];V3L=[2.646;-0.7755;1]; V4L=[-0.384;-0.7755;1];
            V5L = [-0.544;-0.4105;1]; V6L = [-0.544;0.4105;1]; V7L = [-0.384;0.7755;1]; V8L = [2.646;0.7755;1];
            VCL = [1.131;0;1];
            %求车辆八角点在全局坐标系下的位置
            V1G = T*V1L; V2G = T*V2L; V3G = T*V3L; V4G = T*V4L;
            V5G = T*V5L; V6G = T*V6L; V7G = T*V7L; V8G = T*V8L;
            carPose = [V1G(1:2), V2G(1:2), V3G(1:2), V4G(1:2), ...
                V5G(1:2), V6G(1:2), V7G(1:2), V8G(1:2)];
            
            %绘制车辆模型，以长方形框表示实时位置 7
            %             color = 'blue';
            %             linewidth = 0.5;
            %             linestyle = '-';
            %% comment
            set(handles.Trajectory.UserData(7), 'XData', [V1G(1) V2G(1) NaN ...
                V2G(1) V3G(1) NaN V3G(1) V4G(1) NaN ...
                V4G(1) V5G(1) NaN V5G(1) V6G(1) NaN ...
                V6G(1) V7G(1) NaN V7G(1) V8G(1) NaN ...
                V8G(1) V1G(1)], 'Ydata', [V1G(2) V2G(2) NaN ...
                V2G(2) V3G(2) NaN V3G(2) V4G(2) NaN ...
                V4G(2) V5G(2) NaN V5G(2) V6G(2) NaN ...
                V6G(2) V7G(2) NaN V7G(2) V8G(2) NaN ...
                V8G(2) V1G(2)]);
            
            %             plot([V1G(1),V2G(1)],[V1G(2),V2G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            %             hold on
            %             plot([V3G(1),V2G(1)],[V3G(2),V2G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            %             plot([V3G(1),V4G(1)],[V3G(2),V4G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            %             plot([V5G(1),V4G(1)],[V5G(2),V4G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            %             plot([V5G(1),V6G(1)],[V5G(2),V6G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            %             plot([V7G(1),V6G(1)],[V7G(2),V6G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            %             plot([V7G(1),V8G(1)],[V7G(2),V8G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            %             plot([V1G(1),V8G(1)],[V1G(2),V8G(2)],'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
            %             set(handles.Trajectory,'XLim',tr_xlim,'YLim',tr_ylim);
            
        end

        if ~stop && init_flag
            if ~isnan(LocalX(1)) && ~isnan(RefPose1(1))
                %求从全局坐标系到车位坐标系的刚体变换矩阵
%                 % Translate
%                 Tt = [1 0 -p_rl(1); 0 1 -p_rl(2); 0 0 1];
%                 % Rotate
%                 Tr = [cos(PoseTheta) sin(PoseTheta) 0; -sin(PoseTheta) cos(PoseTheta) 0; 0 0 1];
                %求车辆质心在目标车位坐标系下的位置
                V0 = [LocalX;LocalY;1];
                T0_rl = [cos(PoseTheta) -sin(PoseTheta) p_rl(1); sin(PoseTheta) cos(PoseTheta) p_rl(2); 0 0 1];
                V = inv(T0_rl)*V0; %V为车位坐标系下车辆质心位置，车位坐标系以障碍车外角点为原点
                
                %横向偏差
                h = abs(V(2));
                yError = h-0.8;  % positive -- outside
                set(handles.yError,'string',num2str(yError,'%.2f'));
                
                %纵向偏差
                x_= (p_length - 3.57)/2 + 0.544; %标准纵向位置
                xError = V(1) - x_;      % positive -- close to the reference
                set(handles.xError,'string',num2str(xError,'%.2f'));
                
                %航向角偏差
                HeadingAngleError = rad2deg(Yaw-PoseTheta);  % positive -- counterclockwise
                set(handles.HeadingAngleError,'string',num2str(HeadingAngleError,'%.2f'));
                
                

                %             rfp1=[RefPose1(1);RefPose1(2)]; rfp2=[RefPose2(1); RefPose2(2)];
                %             obp1=[ObstaclePose1(1);ObstaclePose1(2)]; obp2=[ObstaclePose2(1);ObstaclePose2(2)];
                [scoreAdd, dangerDistance, collisionFlag] = Risk_Assessment(refPose, obstaclePose, carPose);
                if dangerDistance < minDis && ~isnan(minDis)
                    minDis = dangerDistance;
                end
                risk_score = risk_score + scoreAdd;
                if collisionFlag == 1
                    set(handles.distance, 'String', '与前障碍物碰撞', 'Fontsize', 16, 'ForegroundColor', 'r');
                    minDis = NaN;
                elseif collisionFlag == 2
                    set(handles.distance, 'String', '与后障碍物碰撞', 'Fontsize', 16, 'ForegroundColor', 'r');
                    minDis = NaN;
                else
                    if scoreAdd == 1
                        set(handles.distance, 'String', [num2str(dangerDistance, '%.2f'), ' m'], 'Fontsize', 16, 'ForegroundColor', 'r');
                    else
                        set(handles.distance, 'String', [num2str(dangerDistance, '%.2f'), ' m'], 'Fontsize', 16, 'ForegroundColor', 'k');
                    end
                end
            end
        end
        drawnow limitrate;
    end
%     toc
end


% --- Start parking
function StartStop(handles)
global stop;
global init_flag;

stop = ~stop;
if ~stop
    set(handles.push_start,'String','结束泊车');
    setlog(handles, '泊车开始。');
    setlog(handles, datestr(now));
    set([handles.push_show, handles.ui_score, handles.ui_ScoreDetail, ...
        handles.ui_pk, handles.push_pk],'Visible',0);
    set(handles.push_pk, 'String', '显示人机比拼结果');
    init_flag = 0;
else
    set(handles.push_start,'String','开始泊车');
    setlog(handles, '泊车结束。');
    setlog(handles, datestr(now));
    set(handles.ui_score, 'Visible', 1);
    set(handles.timer,'Visible', 0);
end

% --- Set GUI
function SetGUI(handles)
global tr_xlim
global tr_ylim
global p_width
global displaymethod
global machine

machine = get(handles.St_Robot, 'Value');
set([handles.ui_paraset, handles.push_show], 'Visible', 0);
set([handles.ui_show, handles.ui_tr, handles.ui_msg], 'Visible', 1);
xmin = str2num(get(handles.set_xmin,'String'));
xmax = str2num(get(handles.set_xmax,'String'));
ymin = str2num(get(handles.set_ymin,'String'));
ymax = str2num(get(handles.set_ymax,'String'));
[xmin, xmax] = inorder(xmin, xmax);
[ymin, ymax] = inorder(ymin, ymax);
tr_xlim = [xmin, xmax];
tr_ylim = [ymin, ymax];
p_width = str2num(get(handles.set_pkwidth,'String'));
displaymethod = get(handles.St_show, 'Value');
InitTrajectory(handles);


% --- Reset GUI
function ResetGUI(handles)
global stop;
global flag_loop2;
global PubArrayText;
global h_tr;
global flag_show;
global flag_pk;
global last_Ref;
global machine;

machine = 1;
stop = 1;
flag_loop2 = 0;
h_tr = 0;
flag_show = 0;
flag_pk = 0;
PubArrayText = sprintf('%s\n','');
last_Ref = zeros(12,2);

set(handles.ui_paraset, 'Visible', 1);
set([handles.ui_show,handles.ui_tr, handles.ui_msg, ...
    handles.ui_score, handles.push_show, handles.ui_pk, ...
    handles.push_pk, handles.timer], 'Visible', 0);

set(handles.push_start,'String','开始泊车');
set(handles.push_show,'String','显示轨迹');
set(handles.push_pk, 'String', '显示人机比拼结果');
set(handles.Time, 'String', '');
set(handles.Notice,'String',PubArrayText);

clear_graph(handles);

set([handles.P_length, handles.angle, handles.VehicleSpeed, ...
    handles.Localay, handles.Localax, handles.xError, ...
    handles.yError, handles.HeadingAngleError], 'string','');

% --- Exit GUI
function ExitGUI(handles)
global exit

rosshutdown;
% rmpath('Callbacks');
rmpath('Rank');
setlog(handles, '正在退出......');
exit = 1;

close(gcf);

% --- Show/Hide Trajectory
function ShowTrajectory(handles)
global vehiclePose
global flag_show
global h_tr
global last_Ref

flag_show = ~flag_show;
if flag_show
    setlog(handles, '正在显示......');
    showtrajectory(handles, vehiclePose);
    set(handles.push_show,'String','隐藏轨迹');
else
    setlog(handles,'隐藏轨迹。');
    hidetrajectory(handles, h_tr, last_Ref);
    set(handles.push_show,'String','显示轨迹');
end

function ShowPK(handles)
global flag_pk

flag_pk = ~flag_pk;
if flag_pk
    set(handles.ui_pk, 'Visible', 1);
    set(handles.push_pk, 'String', '隐藏人机比拼结果');
else
    set(handles.ui_pk, 'Visible', 0);
    set(handles.push_pk, 'String', '显示人机比拼结果');
end

% --- Update topic message
function [msgback, vector_indice, flag_update] = getmsg(vector, vector_indice, handles, varargin)
flag_update = 0;
if isempty(varargin)
    if vector.indice > vector_indice
        eval(['set(' 'handles.St_' inputname(1) ', ''Value'', 1)']);
%         set(eval(['handles.St_',inputname(1)]),'Value',1);
        msgback = vector.back();
        vector_indice = vector.indice;
        flag_update = 1;
    else
        eval(['set(' 'handles.St_' inputname(1) ', ''Value'', 0)']);
%         set(eval(['handles.St_',inputname(1)]),'Value',0);
        if vector.indice == 0
            msgback = repelem(NaN, vector.dim);
        else
            msgback = vector.back();
        end
    end
else
    msgback = repelem({NaN},varargin{1});
    if vector{1}.indice > vector_indice
        eval(['set(' 'handles.St_' inputname(1) ', ''Value'', 1)']);
%         set(eval(['handles.St_',inputname(1)]),'Value',1);
        for num = 1 : varargin{1}
            msgback{num} = vector{num}.back();
        end
        vector_indice = vector{1}.indice;
        flag_update = 1;
    else
        eval(['set(' 'handles.St_' inputname(1) ', ''Value'', 0)']);
%         set(eval(['handles.St_',inputname(1)]),'Value',0);
        if vector{1}.indice ~= 0
            for num = 1 : varargin{1}
                msgback{num} = vector{num}.back();
            end
        end
    end
end
    
% --- Display Notice
function setlog(handles, str)
string = sprintf('%s\n',str);
global PubArrayText
PubArrayText = horzcat(string, PubArrayText);
set(handles.Notice,'String',PubArrayText);

% --- Calculate parking time
function time = caltime(t1, t2)
timestr = datestr(t2 - t1, 'HH:MM:SS.FFF');
timevec = datevec(timestr);
time = (timevec(4)*60 + timevec(5))*60 + timevec(6);

% --- Adjust the xlim/ylim order
function [xmin, xmax] = inorder(xmin, xmax)
if xmin > xmax
    t = xmax;
    xmax = xmin;
    xmin = t;
elseif xmin == xmax
    xmax = xmin + 1;
end
    
% --- Calculate the target position
function [p_fl, p_fr, p_rr, p_rl, Rec_l] = calpp(R1, R2, O1, O2, width)
VecL = [R1(1)-O1(1), R1(2)-O1(2)];
VecObs = [O2(1)-O1(1), O2(2)-O1(2)];
VecRef = [R2(1)-R1(1), R2(2)-R1(2)];
theta = atan2(VecL(2), VecL(1));
Flag = [dot(VecL, VecObs) <= 0, dot(-VecL, VecRef) <= 0]; % 1-obtuse angle, 0-acute angle
if isequal(Flag, [1, 1])
    Rec_l = norm(VecL);
    T = [cos(theta), -sin(theta), O1(1); sin(theta), cos(theta), O1(2); 0, 0, 1];
    p_fl = [R1(1); R1(2); 1];
    p_fr = T*[Rec_l; -width; 1];
    p_rr = T*[0; -width; 1];
    p_rl = [O1(1); O1(2); 1];
elseif isequal(Flag, [1, 0])
    Rec_l = norm(VecL) - dot(-VecL, VecRef)/norm(VecL);
    T = [cos(theta), -sin(theta), O1(1); sin(theta), cos(theta), O1(2); 0, 0, 1];
    p_fl = T*[Rec_l; 0; 1];
    p_fr = T*[Rec_l; -width; 1];
    p_rr = T*[0; -width; 1];
    p_rl = [O1(1); O1(2); 1];
elseif isequal(Flag, [0, 0])
    Rec_l = norm(VecL) - (dot(VecL, VecObs) + dot(-VecL, VecRef))/norm(VecL);
    Obsw = abs(det([VecL; VecObs]))/norm(VecL);
    T = [cos(theta), -sin(theta), O2(1); sin(theta), cos(theta), O2(2); 0, 0, 1];
    p_fl = T*[Rec_l; Obsw; 1];
    p_fr = T*[Rec_l; Obsw-width; 1];
    p_rr = T*[0; Obsw-width; 1];
    p_rl = T*[0; Obsw; 1];
else
    Rec_l = norm(VecL) - dot(VecL, VecObs)/norm(VecL);
    T = [cos(theta), -sin(theta), R1(1); sin(theta), cos(theta), R1(2); 0, 0, 1];
    p_fl = [R1(1); R1(2); 1];
    p_fr = T*[0; -width; 1];
    p_rr = T*[-Rec_l; -width; 1];
    p_rl = T*[-Rec_l; 0; 1];
end

% --- initial graphics handles
function [h, n] = initgobj(r,c)
h = gobjects(r,c);
n = [1,1];

% --- plot using handles
function [h, n] = myplot(h, n, x, y)
increase = 1;
if size(h,1) < n(1)
    h = [h; gobjects(increase, size(h,2))];
end
h(n(1),n(2)) = plot(x, y);
n(2) = n(2) + 1;
if n(2) > size(h,2)
    n(2) = 1;
    n(1) = n(1) + 1;
end

function [h, n] = myline(h, n, x, y)
increase = 1;
if size(h,1) < n(1)
    h = [h; gobjects(increase, size(h,2))];
end
h(n(1),n(2)) = line(x, y);
n(2) = n(2) + 1;
if n(2) > size(h,2)
    n(2) = 1;
    n(1) = n(1) + 1;
end

% --- plot polygon(4 or 8 sides), the points should be input according to their order in
% polygon.
% DO NOT USE THIS FUNCTION
function PlotPolygon(handles, num, varargin)
axes(handles.Trajectory);
if nargin == 6
    set(handles.Trajectory.UserData(num), 'XData', [varargin{1}(1) varargin{2}(1) NaN ...
        varargin{2}(1) varargin{3}(1) NaN varargin{3}(1) varargin{4}(1) NaN ...
        varargin{4}(1) varargin{1}(1)], 'Ydata', [varargin{1}(2) varargin{2}(2) NaN ...
        varargin{2}(2) varargin{3}(2) NaN varargin{3}(2) varargin{4}(2) NaN ...
        varargin{4}(2) varargin{1}(2)]);
elseif nargin == 10
    set(handles.Trajectory.UserData(num), 'XData', [varargin{1}(1) varargin{2}(1) NaN ...
        varargin{2}(1) varargin{3}(1) NaN varargin{3}(1) varargin{4}(1) NaN ...
        varargin{4}(1) varargin{5}(1) NaN varargin{5}(1) varargin{6}(1) NaN ...
        varargin{6}(1) varargin{7}(1) NaN varargin{7}(1) varargin{8}(1) NaN ...
        varargin{8}(1) varargin{1}(1)], 'Ydata', [varargin{1}(2) varargin{2}(2) NaN ...
        varargin{2}(2) varargin{3}(2) NaN varargin{3}(2) varargin{4}(2) NaN ...
        varargin{4}(2) varargin{5}(2) NaN varargin{5}(2) varargin{6}(2) NaN ...
        varargin{6}(2) varargin{7}(2) NaN varargin{7}(2) varargin{8}(2) NaN ...
        varargin{8}(2) varargin{1}(2)]);
else
    disp('PlotPolygon parameters number error.');
end


% --- show trajectory
function showtrajectory(handles, vehiclePose)
global h_tr

if ~get(handles.St_show, 'Value')
    %% use handles
    
    if h_tr ~= 0
        set(h_tr, 'Visible', 1);
    else
        data = vehiclePose.get_data();
        NumberofTr = [1, 1];
        [h_tr, n_tr] = initgobj(NumberofTr(1),NumberofTr(2));
        axes(handles.Trajectory)
        %求车辆八角点在车身坐标系下的位置
        V1L = [3.026;0.3955;1];V2L=[3.026;-0.3955;1];V3L=[2.646;-0.7755;1]; V4L=[-0.384;-0.7755;1];
        V5L = [-0.544;-0.4105;1]; V6L = [-0.544;0.4105;1]; V7L = [-0.384;0.7755;1]; V8L = [2.646;0.7755;1];
        color = 'blue';
        linewidth = 0.5;
        linestyle = '-';
        for n = 1 : 10 : size(data, 1)
            T = [cos(data(n,4)), -sin(data(n,4)), data(n,2);...
                sin(data(n,4)), cos(data(n,4)), data(n,3); 0, 0, 1];
            %求车辆八角点在全局坐标系下的位置
            V1G = T*V1L; V2G = T*V2L; V3G = T*V3L; V4G = T*V4L;
            V5G = T*V5L; V6G = T*V6L; V7G = T*V7L; V8G = T*V8L;
            % Draw
            myline(h_tr, n_tr, [V1G(1) V2G(1) NaN V2G(1) V3G(1) NaN V3G(1) V4G(1) NaN V4G(1) V5G(1) NaN ...
            V5G(1) V6G(1) NaN V6G(1) V7G(1) NaN V7G(1) V8G(1) NaN V8G(1) V1G(1)], ...
            [V1G(2) V2G(2) NaN V2G(2) V3G(2) NaN V3G(2) V4G(2) NaN V4G(2) V5G(2) NaN ...
            V5G(2) V6G(2) NaN V6G(2) V7G(2) NaN V7G(2) V8G(2) NaN V8G(2) V1G(2)])
%             hold on
%             [h_tr, n_tr] = myplot(h_tr, n_tr, [V1G(1),V2G(1)],[V1G(2),V2G(2)]);
%             [h_tr, n_tr] = myplot(h_tr, n_tr, [V3G(1),V2G(1)],[V3G(2),V2G(2)]);
%             [h_tr, n_tr] = myplot(h_tr, n_tr, [V3G(1),V4G(1)],[V3G(2),V4G(2)]);
%             [h_tr, n_tr] = myplot(h_tr, n_tr, [V5G(1),V4G(1)],[V5G(2),V4G(2)]);
%             [h_tr, n_tr] = myplot(h_tr, n_tr, [V5G(1),V6G(1)],[V5G(2),V6G(2)]);
%             [h_tr, n_tr] = myplot(h_tr, n_tr, [V7G(1),V6G(1)],[V7G(2),V6G(2)]);
%             [h_tr, n_tr] = myplot(h_tr, n_tr, [V7G(1),V8G(1)],[V7G(2),V8G(2)]);
%             [h_tr, n_tr] = myplot(h_tr, n_tr, [V1G(1),V8G(1)],[V1G(2),V8G(2)]);
        end
        set(h_tr(1:n_tr(1)-1, 1:size(h_tr,2)), 'Color', color, 'LineWidth', linewidth, 'LineStyle', linestyle);
    end
    
else
    %% abort handles
    data = vehiclePose.get_data();
    axes(handles.Trajectory)
    %求车辆八角点在车身坐标系下的位置
    V1L = [3.026;0.3955;1];V2L=[3.026;-0.3955;1];V3L=[2.646;-0.7755;1]; V4L=[-0.384;-0.7755;1];
    V5L = [-0.544;-0.4105;1]; V6L = [-0.544;0.4105;1]; V7L = [-0.384;0.7755;1]; V8L = [2.646;0.7755;1];
    color = 'blue';
    linewidth = 0.5;
    linestyle = '-';
    for n = 1 : 10 : size(data, 1)
        T = [cos(data(n,4)), -sin(data(n,4)), data(n,2);...
            sin(data(n,4)), cos(data(n,4)), data(n,3); 0, 0, 1];
        %求车辆八角点在全局坐标系下的位置
        V1G = T*V1L; V2G = T*V2L; V3G = T*V3L; V4G = T*V4L;
        V5G = T*V5L; V6G = T*V6L; V7G = T*V7L; V8G = T*V8L;
%         hold on
%         plot([V1G(1) V2G(1) NaN V2G(1) V3G(1) NaN V3G(1) V4G(1) NaN V4G(1) V5G(1) NaN ...
%             V5G(1) V6G(1) NaN V6G(1) V7G(1) NaN V7G(1) V8G(1) NaN V8G(1) V1G(1)], ...
%             [V1G(2) V2G(2) NaN V2G(2) V3G(2) NaN V3G(2) V4G(2) NaN V4G(2) V5G(2) NaN ...
%             V5G(2) V6G(2) NaN V6G(2) V7G(2) NaN V7G(2) V8G(2) NaN V8G(2) V1G(2)], ...
%             'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
        line([V1G(1) V2G(1) NaN V2G(1) V3G(1) NaN V3G(1) V4G(1) NaN V4G(1) V5G(1) NaN ...
            V5G(1) V6G(1) NaN V6G(1) V7G(1) NaN V7G(1) V8G(1) NaN V8G(1) V1G(1)], ...
            [V1G(2) V2G(2) NaN V2G(2) V3G(2) NaN V3G(2) V4G(2) NaN V4G(2) V5G(2) NaN ...
            V5G(2) V6G(2) NaN V6G(2) V7G(2) NaN V7G(2) V8G(2) NaN V8G(2) V1G(2)], ...
            'Color',color,'LineWidth', linewidth, 'LineStyle', linestyle);
    end
    
end
setlog(handles, '显示完毕。');

% --- hide trajectory
function hidetrajectory(handles, h, Ref)
Ref = Ref';
if h == 0
    InitTrajectory(handles);
    
    %绘制算法目标车位  3
    axes(handles.Trajectory);
    PlotPolygon(handles, 3, Ref(1,:), Ref(2,:), Ref(3,:), Ref(4,:));
    
    %绘制实际泊车空间  4
    PlotPolygon(handles, 4, Ref(7,:), Ref(8,:), Ref(9,:), Ref(10,:));
    
    %绘制障碍物方块  5, 6
    PlotPolygon(handles, 5, Ref(5,:), Ref(6,:), Ref(7,:), Ref(8,:));
    PlotPolygon(handles, 6, Ref(9,:), Ref(10,:), Ref(11,:), Ref(12,:));
    
    %填充  1, 2
    set(handles.Trajectory.UserData(1), 'XData', [Ref(5,1), Ref(6,1), Ref(7,1), Ref(8,1)], ...
        'YData', [Ref(5,2), Ref(6,2), Ref(7,2), Ref(8,2)]);
    set(handles.Trajectory.UserData(2), 'XData', [Ref(9,1), Ref(10,1), Ref(11,1), Ref(12,1)], ...
        'YData', [Ref(9,2), Ref(10,2), Ref(11,2), Ref(12,2)]);
    
else
    set(h,'Visible',0);
end
%车辆 7
PlotPolygon(handles, 7, Ref(13,:), Ref(14,:), Ref(15,:), Ref(16,:),Ref(17,:), Ref(18,:), Ref(19,:), Ref(20,:));


% --- Clear graph
function clear_graph(handles)

% axes(handles.sc1); cla(handles.sc1); set(handles.sc1, 'XTick', {}, 'YTick', {});
set(handles.timeScore, 'string', ' ');
set(handles.accScore, 'string', ' ');
set(handles.safetyScore, 'string', ' ');
set(handles.comScore, 'string', ' ');
set(handles.rotScore, 'string', ' ');

InitTrajectory(handles);


% --- Trajectory Display Initialize
function InitTrajectory(handles)
global tr_xlim;  % xlim of handles.Trajectory
global tr_ylim;  % ylim of handles.Trajectory

axes(handles.Trajectory);
cla(handles.Trajectory);
hold on
set(handles.Trajectory, 'UserData', [patch([0 0 0 0],[0 0 0 0],'k'), patch([0 0 0 0],[0 0 0 0],'k'), ... fill
    line([0 0],[0 0],'Color','red','LineWidth',0.5,'LineStyle','-'), ... target
    line([0 0],[0 0],'Color','green','LineWidth',0.5,'LineStyle','-'), ... parking slot
    line([0 0],[0 0],'Color','black','LineWidth',0.5,'LineStyle','-'), ... obstacle
    line([0 0],[0 0],'Color','black','LineWidth',0.5,'LineStyle','-'), ... 
    line([0 0],[0 0],'Color','blue','LineWidth',0.5,'LineStyle','-')]);   % vehicle
hold off
set(handles.Trajectory,'XLim',tr_xlim,'YLim',tr_ylim);

%--- CallBack
function Vehicle_pose2DCallback(~,message,buffer)
    LocalX = message.Pose.Pose.Position.X;
    LocalY = message.Pose.Pose.Position.Y;
    q = [message.Pose.Pose.Orientation.W message.Pose.Pose.Orientation.X ...
        message.Pose.Pose.Orientation.Y message.Pose.Pose.Orientation.Z];
    EulAngle = quat2eul(q);

    buffer.push_back([message.Header.Stamp.seconds,LocalX,LocalY,EulAngle(1)])

function parkingslotCallback(~,message,buffer)
        
RefPoseX = message.RefPose.X;
RefPoseY = message.RefPose.Y;
RefPoseTheta = message.RefPose.Theta;
RefExtend_X = message.RefExtendX;
RefExtend_Y = message.RefExtendY;

T1 = [cos(RefPoseTheta), -sin(RefPoseTheta), RefPoseX; sin(RefPoseTheta), cos(RefPoseTheta), RefPoseY; 0, 0, 1];
RefPose1 = [RefPoseX;RefPoseY;1];
b1 = [ 0; RefExtend_Y; 1];
c1 = [ RefExtend_X;0;1];
d1 = [ RefExtend_X;RefExtend_Y; 1];
RefPose2 = T1*b1;
RefPose3 = T1*c1;
RefPose4 = T1*d1;

ObstaclePoseX = message.ObstaclePose.X;
ObstaclePoseY = message.ObstaclePose.Y;
ObstacleTheta = message.ObstaclePose.Theta;
ObstacleExtend_X = message.ObstacleExtendX;
ObstacleExtend_Y = message.ObstacleExtendY;
T2 = [cos(ObstacleTheta), -sin(ObstacleTheta), ObstaclePoseX; sin(ObstacleTheta), cos(ObstacleTheta), ObstaclePoseY; 0, 0, 1];
ObstaclePose1 = [ObstaclePoseX,ObstaclePoseY];
b2 = [ 0; ObstacleExtend_Y; 1];
c2 = [ ObstacleExtend_X;0;1];
d2 = [ ObstacleExtend_X;ObstacleExtend_Y; 1];
ObstaclePose2 = T2*b2;
ObstaclePose3 = T2*c2;
ObstaclePose4 = T2*d2;

buffer.push_back([message.Header.Stamp.seconds,RefPose1(1),RefPose1(2), ...
    RefPose2(1),RefPose2(2),RefPose3(1),RefPose3(2),RefPose4(1),RefPose4(2), ...
    ObstaclePose1(1), ObstaclePose1(2),ObstaclePose2(1), ObstaclePose2(2), ...
    ObstaclePose3(1), ObstaclePose3(2),ObstaclePose4(1), ObstaclePose4(2), ...
    RefPoseTheta])

function imuCallback(~,message,buffer)
Ax = message.LinearAcceleration.X;
Ay = message.LinearAcceleration.Y;
buffer.push_back([message.Header.Stamp.seconds, Ax, Ay])


function SteeringAngleCallback(~,message,buffer)
buffer.push_back([message.Header.Stamp.seconds, -message.Angle]);

function velometerCallback(~,message,buffer)

Vx = message.Twist.Linear.X;
Vy = message.Twist.Linear.Y;
Speed=sqrt(Vx*Vx+Vy*Vy);
buffer.push_back([message.Header.Stamp.seconds, Speed]);



% --- Executes on button press in push_paraset.
function push_paraset_Callback(hObject, eventdata, handles)
% hObject    handle to push_paraset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Xlim & Ylim for trajectory display    
    
if get(handles.push_paraset, 'value')
    SetGUI(handles);
end



% --- Executes on button press in push_start.
function push_start_Callback(hObject, eventdata, handles)
% hObject    handle to push_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if get(handles.push_start, 'value')
    StartStop(handles);
end

% --- Executes on button press in push_exit.
function push_exit_Callback(hObject, eventdata, handles)
% hObject    handle to push_exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if get(handles.push_exit, 'value')
    ExitGUI(handles);
end



% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
delete(hObject);

function set_xmin_Callback(hObject, eventdata, handles)
% hObject    handle to set_xmin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_xmin as text
%        str2double(get(hObject,'String')) returns contents of set_xmin as a double


% --- Executes during object creation, after setting all properties.
function set_xmin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_xmin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function set_xmax_Callback(hObject, eventdata, handles)
% hObject    handle to set_xmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_xmax as text
%        str2double(get(hObject,'String')) returns contents of set_xmax as a double


% --- Executes during object creation, after setting all properties.
function set_xmax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_xmax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function set_ymin_Callback(hObject, eventdata, handles)
% hObject    handle to set_ymin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_ymin as text
%        str2double(get(hObject,'String')) returns contents of set_ymin as a double


% --- Executes during object creation, after setting all properties.
function set_ymin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_ymin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function set_ymax_Callback(hObject, eventdata, handles)
% hObject    handle to set_ymax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_ymax as text
%        str2double(get(hObject,'String')) returns contents of set_ymax as a double


% --- Executes during object creation, after setting all properties.
function set_ymax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_ymax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in push_reset.
function push_reset_Callback(hObject, eventdata, handles)
% hObject    handle to push_reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if get(handles.push_reset, 'value')
    ResetGUI(handles);
end



function set_pkwidth_Callback(hObject, eventdata, handles)
% hObject    handle to set_pkwidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_pkwidth as text
%        str2double(get(hObject,'String')) returns contents of set_pkwidth as a double


% --- Executes during object creation, after setting all properties.
function set_pkwidth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_pkwidth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in push_show.
function push_show_Callback(hObject, eventdata, handles)
% hObject    handle to push_show (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if get(handles.push_show, 'value')
    ShowTrajectory(handles);
end


% --- Executes on button press in St_show.
function St_show_Callback(hObject, eventdata, handles)
% hObject    handle to St_show (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_show


% --- Executes on button press in push_close.
function push_close_Callback(hObject, eventdata, handles)
% hObject    handle to push_close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if get(handles.push_close, 'value')
    ExitGUI(handles);
end



% --- Executes on button press in St_save.
function St_save_Callback(hObject, eventdata, handles)
% hObject    handle to St_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_save


% --- Executes on button press in St_parkingSlot.
function St_parkingSlot_Callback(hObject, eventdata, handles)
% hObject    handle to St_parkingSlot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_parkingSlot


% --- Executes on button press in St_vehiclePose.
function St_vehiclePose_Callback(hObject, eventdata, handles)
% hObject    handle to St_vehiclePose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_vehiclePose



function set_name_Callback(hObject, eventdata, handles)
% hObject    handle to set_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of set_name as text
%        str2double(get(hObject,'String')) returns contents of set_name as a double


% --- Executes during object creation, after setting all properties.
function set_name_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_name (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on figure1 or any of its controls.
function figure1_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

% try
%     myKey = double(get(gcf,'CurrentCharacter'));
% catch 
% end

% if ~isempty(myKey)
%     switch myKey
%         case 13   % enter
%             if get(handles.ui_show,'Visible')
%                 StartStop(handles);
%             end
%         case 27   % esc
%             ExitGUI(handles);
%         case 98   % 'b'
%             if get(handles.ui_paraset,'Visible')
%                 set(handles.St_Robot, 'Value', ~handles.St_Robot.Value);
%                 set(handles.set_name, 'String', '机器');
%             end
%         case 100  % 'd'
%             if get(handles.ui_paraset,'Visible')
%                 set(handles.St_save, 'Value', ~handles.St_save.Value);
%             end
%         case 113  % 'q'
%             set(handles.ui_Debug, 'Visible', ~handles.ui_Debug.Visible);
%         case 114  % 'r'
%             if get(handles.ui_show,'Visible')
%                 ResetGUI(handles);
%             end
%         case 115  % 's'
%             if get(handles.ui_paraset,'Visible')
%                 SetGUI(handles);
%             end
%         case 116 % 't'
%             if get(handles.ui_show,'Visible')
%                 ShowTrajectory(handles);
%             end
%         otherwise
%     end
% end


% --- Executes on button press in St_Robot.
function St_Robot_Callback(hObject, eventdata, handles)
% hObject    handle to St_Robot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_Robot
if get(handles.St_Robot, 'Value') == 1
    set(handles.set_name, 'String', '机器');
end

% --- Executes on button press in St_Debug.
function St_Debug_Callback(hObject, eventdata, handles)
% hObject    handle to St_Debug (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_Debug


% --- Executes on button press in timer.
function timer_Callback(hObject, eventdata, handles)
% hObject    handle to timer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of timer


% --- Executes on button press in push_pk.
function push_pk_Callback(hObject, eventdata, handles)
% hObject    handle to push_pk (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if get(handles.push_pk, 'Value') == 1
    ShowPK(handles);
end


% --- Executes on button press in St_gear.
function St_gear_Callback(hObject, eventdata, handles)
% hObject    handle to St_gear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_gear


% --- Executes on button press in St_en.
function St_en_Callback(hObject, eventdata, handles)
% hObject    handle to St_en (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of St_en


% --- Executes on button press in distance.
function distance_Callback(hObject, eventdata, handles)
% hObject    handle to distance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of distance
