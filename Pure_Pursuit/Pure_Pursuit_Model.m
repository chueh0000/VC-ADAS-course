clc
clear
close all
%% 設計道路
straight1x=(0:0.1:20)';%第一條直線x值
straight1y=2.*ones(size(straight1x));%第一條直線y值
[Corner1x, Corner1y, Corner2x, Corner2y] = DoubleCornerPoint(straight1x,straight1y);%兩條圓弧的x,y值，用DoubleCornerPoint計算
straight2x=(Corner2x(end):0.1:Corner2x(end)+30)';%第二條直線x值
straight2y=Corner2y(end).*ones(size(straight2x));%第二條直線y值

RoadData=horzcat([straight1x;Corner1x;Corner2x;straight2x], ...
    [straight1y;Corner1y;Corner2y;straight2y]);%道路資料點
%% 車輛參數
m   = 1270 + 88.6 + 54.4;  % 總質量 [kg]
Iz  = 1536.7;              % 偏航慣性矩 [kg*m^2]
Caf = 46000 * 2;           % 前輪側偏剛度 [N/rad]
Car = 42000 * 2;           % 後輪側偏剛度 [N/rad]
l_tot = 2.91;              % 軸距 [m]
la = 1.015;                % 前軸到質心距離 [m]
lb = l_tot - la;           % 質心到後軸距離 [m]

u0 = 5;                    % 縱向速度 [m/s]
LookAheadTimes=[1, 2];     % 前視時間

% 建立儲存空間來保存每次模擬的結果
X_results = cell(1, length(LookAheadTimes));
Y_results = cell(1, length(LookAheadTimes));

%% 車輛尺寸(動畫用)
vehicle_length = 4.5;   % 車輛長度 [m]
vehicle_width = 2.082;  % 車輛寬度 [m]
wheel_radius = 0.4;     % 車輪半徑 [m]
wheel_width = 0.2;      % 車輪寬度 [m]
%% 車輛狀態空間模型
% A矩陣
Av4 = [0, 1, u0, 0;
       0, -(Caf+Car)/(m*u0), 0, (lb*Car - la*Caf)/(m*u0) - u0;
       0, 0, 0, 1;
       0, (lb*Car - la*Caf)/(Iz*u0), 0, -(la^2*Caf + lb^2*Car)/(Iz*u0)];
% B矩陣
Bv4 = [0;
       Caf / m;
       0;
       la * Caf / Iz];

% C矩陣
Cv4 = eye(4);

% D矩陣
Dv4 = zeros(4,1);

%% 模擬參數
sim_hz=100;         % 模擬採樣頻率 [hz]
input_hz=2;         % 輸入頻率 [hz]
t_total = 15;       % 總模擬時間 [s]
dt = 1/sim_hz;      % 模擬時間步長 [s]
input_dt=1/input_hz;% 輸入時間步長 [s]
t = 0:dt:t_total;   % 模擬時間序列
n_steps = length(t);% 細分後模擬時間數量
%% 駕駛控制參數
delay=0.02;
steer_angle_delay = zeros(delay/dt,1);
steer_angle_deadzone=[deg2rad(-0.5),deg2rad(0.5)];
gain=1;


%% 比較循環 (Comparison Loop)
for sim_idx = 1:length(LookAheadTimes)
    
    % 1. 設置當前的前視參數
    LookAheadTime = LookAheadTimes(sim_idx);
    LookAheadDis = u0 * LookAheadTime;  % 前視範圍，當下速度過前視時間秒後的所在位置
    
    % 2. 初始化 (Reset conditions for every new run)
    x0 = [0; 0; 0; 0];      % 初始狀態：[橫向位置；橫向速度；偏航角；偏航角速度]
    x = zeros(4, n_steps);  % 儲存狀態變量
    x(:,1) = x0;
    
    X = 0;                  % 全局坐標系下的質心x位置
    Y = 0;                  % 全局坐標系下的質心y位置
    
    X_front(1) = la;        % 全局坐標系下的前軸x位置
    Y_front = 0;            % 全局坐標系下的前軸y位置
    
    X_rear(1) = -lb;        % 全局坐標系下的後軸x位置
    Y_rear = 0;             % 全局坐標系下的後軸y位置
    CarrotPoint=[0,0];      % 蘿蔔點初始值
    st_n=0;

    steer_angle_delay = zeros(delay/dt,1); % Reset delay buffer

    % 3. 模擬循環
    for k = 1:n_steps-1
        %% 搜尋道路範圍並找到蘿蔔點
        % 找出道路在車輛可視範圍內的所有數據點
        upx=LookAheadDis+X(k);
        upy=LookAheadDis+Y(k);
        lowx=X(k)-LookAheadDis;
        lowy=Y(k)-LookAheadDis;
    
        Lookindex = RoadData(:,1) >= lowx & RoadData(:,1) <= upx & RoadData(:,2) >= lowy & RoadData(:,2) <= upy;
       
        % 使用這個索引來選擇對應的行
        CanLookData = RoadData(Lookindex, :);
    
        % 計算分割點
        splitPoint = ceil(size(CanLookData, 1) / 2);  % 使用 ceil 確保即使不是偶數也能正確分割
    
        % 只保留後半段
        NowLookData = CanLookData(splitPoint+1:end, :);
    
        LadVchdistances = zeros(size(NowLookData, 1), 1); % 初始化一個與 RoadData 行數相同的數組，用來儲存每個距離
    
        for i = 1:size(NowLookData, 1)
            x_diff = NowLookData(i, 1) - X(k); % 計算 x 方向的差值
            y_diff = NowLookData(i, 2) - Y(k); % 計算 y 方向的差值
            LadVchdistances(i) = sqrt(x_diff^2 + y_diff^2); % 計算距離並存入數組
        end
    
        [~, minpath] = min(abs(LadVchdistances - LookAheadDis)); % 找出與目標距離差值最小的索引，NowLookData(minpath,1)是蘿蔔點的x值,NowLookData(minpath,2)是蘿蔔點的y值
        %% 計算轉向角
        % 定義向量
        LADv = [NowLookData(minpath,1)-X_rear(k),NowLookData(minpath,2)-Y_rear(k)];%從車輛後軸點到蘿蔔點的向量
        Vehv = [X_front(k)-X_rear(k), Y_front(k)-Y_rear(k)];%從車輛後軸點到車輛前軸點的向量
    
        % 計算點積
        dotLADvVehv = dot(LADv, Vehv);
    
        % 計算向量長度
        length_LADv = norm(LADv);
        length_Vehv = norm(Vehv);
    
        % 計算夾角的餘弦值
        LADvVehv_theta =dotLADvVehv / (length_LADv * length_Vehv);
    
        % 計算夾角
        LADvVehv_angle = acos(LADvVehv_theta);%取反cos來求兩向量的夾角
    
        LADv_alpha=pi-(2*(pi/2-LADvVehv_angle));%等腰三角形特性，用來求頂角
        CornerRNow=LadVchdistances(minpath)/(2*sin((LADv_alpha/2)));%這就是當下的轉彎半徑
    
        %下面這個副函式是用來找出目前轉向的方向，1是左轉,-1是右轉
        steerpath=judgePosition(X_rear(k), Y_rear(k), X_front(k), Y_front(k), NowLookData, minpath);
    
        steer_angle=atan(l_tot/CornerRNow)*steerpath;%這就是轉向角
    
        st(k+1,:)=steer_angle*180/pi;%紀錄轉向角
        %% 駕駛員因素
        if steer_angle>=steer_angle_deadzone(1)&&steer_angle<=steer_angle_deadzone(2)
        steer_angle=0;
        end
        steer_angle_delay(end+1)=steer_angle*gain;
        %%
        % 計算狀態導數
        x_dot = Av4 * x(:,k) + Bv4 * steer_angle_delay(k);
        
        % 更新狀態
        x(:,k+1) = x(:,k) + x_dot * dt;
        % 提取偏航角和橫向速度
        psi = x(3,k);      % 偏航角
        vy = x(2,k);       % 橫向速度
        
        % 計算全局坐標系下的速度
        Vx = u0 * cos(psi) - vy * sin(psi);
        Vy = u0 * sin(psi) + vy * cos(psi);
        
        % 更新質心位置
        X(k+1) = X(k) + Vx * dt;
        Y(k+1) = Y(k) + Vy * dt;
        
        % 計算前軸位置
        X_front(k+1) = X(k+1) + la * cos(psi);
        Y_front(k+1) = Y(k+1) + la * sin(psi);
        
        % 計算後軸位置
        X_rear(k+1) = X(k+1) - lb * cos(psi);
        Y_rear(k+1) = Y(k+1) - lb * sin(psi);
        CarrotPoint(k+1,:)=NowLookData(minpath,:);%紀錄蘿蔔點
        st_n(k+1,:)=steer_angle_delay(k);
    end

    % 4. 儲存這次模擬的質心軌跡
    X_results{sim_idx} = X;
    Y_results{sim_idx} = Y;
end

%% 計算圖表大小
% 獲取螢幕解析度
screenResolution = get(0, 'ScreenSize');
screenWidth = screenResolution(3);
screenHeight = screenResolution(4);
% 計算標題高度
titleHeight = 30; 

% 計算圖表視窗位置與大小
figureWidth = (screenWidth);
figureHeight = (screenHeight/2) - titleHeight; 
figurePosition = [1, titleHeight, figureWidth, figureHeight];
%% 創建動畫
figure('Position', figurePosition);
plot(RoadData(:,1),RoadData(:,2))
hold on
for k = 1:10:n_steps
    clf;%清除上一動
    %% 繪製到當前時間的質心、前軸和後軸軌跡
    plot(X(1:k), Y(1:k), 'b-', 'LineWidth', 1); % 質心軌跡
    hold on;
    plot(X_front(1:k), Y_front(1:k), 'g--', 'LineWidth', 1); % 前軸軌跡
    plot(X_rear(1:k), Y_rear(1:k), 'r--', 'LineWidth', 1);   % 後軸軌跡
    plot(RoadData(:,1),RoadData(:,2))
    % 提取當前偏航角
    psi = x(3,k); % 當前時刻的偏航角
    %% 繪製車輛車身
    % 計算車輛矩形的角點
    % 車輛中心為 (X(k), Y(k))，方向為 psi
    corner_offsets = [ vehicle_length/2,  vehicle_width/2;
                       vehicle_length/2, -vehicle_width/2;
                      -vehicle_length/2, -vehicle_width/2;
                      -vehicle_length/2,  vehicle_width/2 ]';

    % 旋轉矩陣
    R = [cos(psi), -sin(psi);
         sin(psi),  cos(psi)];

    % 旋轉角點
    rotated_corners = R * corner_offsets;

    % 平移到當前位置
    corners_x = rotated_corners(1,:) + X(k);
    corners_y = rotated_corners(2,:) + Y(k);

    % 繪製車輛矩形
    fill(corners_x, corners_y, [0.8 0.8 0.8]);
    %% 繪製當前車輛質心位置
    plot(X(k), Y(k), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b'); % 質心位置
    %% 繪製當前前軸和後軸位置
    plot(X_front(k), Y_front(k), 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g'); % 前軸位置
    plot(X_rear(k), Y_rear(k), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');   % 後軸位置
    %% 繪製蘿蔔點
    plot([CarrotPoint(k, 1), X(k)], [CarrotPoint(k, 2), Y(k)],'rx-')
    %% 處裡圖表
    xlabel('X 位置 [m]');
    ylabel('Y 位置 [m]');
    title('Pure Pursuit Model');
    %legend('質心軌跡', '前軸軌跡', '後軸軌跡', 'Location', 'Best');
    grid on;
    axis equal;
    axis([min(X)-10, max(X)+10, min(Y)-10, max(Y)+10]);
    pause(0.01);
end


%% 繪製軌跡
figure
hold on
% plot(RoadData(:,1),RoadData(:,2),'k-', X, Y, 'b-', X_front, Y_front, 'g--', X_rear, Y_rear, 'r--', 'LineWidth', 1);

% plot(RoadData(:,1),RoadData(:,2),'k-', X, Y, 'b-', 'LineWidth', 1);

plot(RoadData(:,1), RoadData(:,2), 'k-', 'LineWidth', 1.5);  % 繪製道路 (Black line)
plot(X_results{1}, Y_results{1}, 'b-', 'LineWidth', 1.2); % 繪製 LookAheadTime = 1 (Blue solid line)
plot(X_results{2}, Y_results{2}, 'r--', 'LineWidth', 1.2);  % 繪製 LookAheadTime = 2 (Red dashed line)

xlabel('X 位置 [m]');
ylabel('Y 位置 [m]');
% title('車輛質心、前軸和後軸軌跡');
title('Pure Pursuit: Look-Ahead Time 比較 (1秒 vs 2秒)');
% legend('道路','質心軌跡', '前軸軌跡', '後軸軌跡');
% legend('道路','質心軌跡');
legend('道路 (Path)', 'LookAheadTime = 1s', 'LookAheadTime = 2s');
% aa=readmatrix("teashrg.csv");
% plot(aa(:,1),aa(:,2))
grid on;
axis equal
hold off



% figure
% plot(t,rad2deg(st_n))
%% 副函式:CornerCenterNow-找到當下過彎中心(目前非必要)


function CornerCenterNow=circle_intersections(vehicleRear0x,vehicleRear0y,NowLookData,minpath,CornerRNow,vehicleFront0x,vehicleFront0y)
% 定義圓的參數
x1 = vehicleRear0x;
y1 = vehicleRear0y;
r1 = CornerRNow;
x2 = NowLookData(minpath,1);
y2 = NowLookData(minpath,2);
r2 = CornerRNow;

% 圓方程差的線性化，用於求解x和y
syms x y;
eq1 = x^2 + y^2 == r1^2;
eq2 = (x - x2)^2 + (y - y2)^2 == r2^2;

% 解方程
solutions = solve([eq1, eq2], [x, y], 'Real', true);

% 比較哪個點離前軸點更近
point = [vehicleFront0x, vehicleFront0y];
dist1 = norm([double(solutions.x(1)) - point(1), double(solutions.y(1)) - point(2)]);
dist2 = norm([double(solutions.x(2)) - point(1), double(solutions.y(2)) - point(2)]);

    if dist1 > dist2
       CornerCenterNow=[double(solutions.x(1)), double(solutions.y(1))];
    else
       CornerCenterNow=[double(solutions.x(2)), double(solutions.y(2))];
    end
end

%% 副函式:cornerpoint-計算彎道數據點
function [Corner1x_points,Corner1y_points,Corner2x_points,Corner2y_points]=DoubleCornerPoint(straight1x,straight1y)
% 圓弧1 和圓弧2 的參數

CornerR1 = 30;CornerCenter1x = straight1x(end); CornerCenter1y = CornerR1+straight1y(end);% 圓弧1
CornerR2 = 30; CornerEndy = 10; CornerCenter2y = CornerEndy - CornerR2; % 圓弧2

% 計算圓弧2的圓心x坐標
syms CornerCenter2x;
Cornereq = (CornerCenter1x - CornerCenter2x)^2 + (CornerCenter1y - CornerCenter2y)^2 == (CornerR1 + CornerR2)^2;
sol_CornerCenter2x = double(solve(Cornereq, CornerCenter2x));
CornerCenter2x = sol_CornerCenter2x(sol_CornerCenter2x > CornerCenter1x);

% 相切點角度
tangent_angle = atan2(CornerCenter2y - CornerCenter1y, CornerCenter2x - CornerCenter1x);

% 設定資料點數量
N = 100;

% 圓弧1 的角度範圍和資料點
theta1_start = -pi/2;  % 圓心正下方
theta1_end = tangent_angle;  % 與圓弧2的相切點
Cornertheta1 = linspace(theta1_start, theta1_end, N);
Corner1x_points = (CornerCenter1x + CornerR1 * cos(Cornertheta1))';
Corner1y_points = (CornerCenter1y + CornerR1 * sin(Cornertheta1))';

% 圓弧2 的角度範圍和資料點
theta2_start = pi-abs(tangent_angle);  % 與圓弧1的相切點
theta2_end = pi/2;  % 圓心正上方
Cornertheta2 = linspace(theta2_start, theta2_end, N);
Corner2x_points = (CornerCenter2x + CornerR2 * cos(Cornertheta2))';
Corner2y_points = (CornerCenter2y + CornerR2 * sin(Cornertheta2))';

end
%% 副函式:judgePosition-找出目前轉向的方向
function position = judgePosition(vehicleRear0x, vehicleRear0y, vehicleFront0x, vehicleFront0y, NowLookData, minpath)
xA=vehicleRear0x;yA=vehicleRear0y;
xB=vehicleFront0x;yB=vehicleFront0y;
xC=NowLookData(minpath,1); yC=NowLookData(minpath,2);
vectorAB = [xB - xA, yB - yA];
vectorAC = [xC - xA, yC - yA];
crossProduct = vectorAB(1) * vectorAC(2) - vectorAB(2) * vectorAC(1);
    if crossProduct > 0
        position = 1;  % C點在AB的左邊
    elseif crossProduct < 0
       position = -1; % C點在AB的右邊
    else
       position = 0;  % C點在AB線上
    end
end

%% 副函式:計算圓弧數據點
function [Corner3x,Corner3y]=SingleCornerPoint(straight2x,straight2y,r)
% 圓參數
x_center = straight2x(end);
y_center = straight2y(end)+r;


% 點的數量 (例如 10 個點)
n = 100;

% 起始點角度
theta_0 = -pi/2;  % 從(80, 10)開始

% 計算每個點的座標
theta = theta_0 + (0:n-1) * (pi / n);  % 均勻分布的角度
Corner3x = (x_center + r * cos(theta))';  % x 坐標
Corner3y = (y_center + r * sin(theta))';  % y 坐標
end