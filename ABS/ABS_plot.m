% Make sure your model name matches exactly what is in the top left of the explorer
model_name = 'ABS'; 
load_system(model_name); % Load model into memory

% Get direct access to the Model Workspace
mdlWks = get_param(model_name, 'ModelWorkspace');

%% 1. Run Simulation: ABS OFF
disp('Running ABS OFF...');
mdlWks.assignin('ABS_mode', 0); % Physically change the value in Model Explorer to 0
out_off = sim(model_name);

% Extract Data (Assuming the Mux order is Slip, Wheel Speed, Distance, Vehicle Speed)
time_off          = out_off.tout;
slip_off          = out_off.state.Data(:, 1); 
wheel_speed_off   = out_off.state.Data(:, 2); 
dist_off          = out_off.state.Data(:, 3); 
vehicle_speed_off = out_off.state.Data(:, 4); 

%% 2. Run Simulation: ABS ON
disp('Running ABS ON...');
mdlWks.assignin('ABS_mode', 1); % Physically change the value in Model Explorer to 1
out_on = sim(model_name);

% Extract Data
time_on          = out_on.tout;
slip_on          = out_on.state.Data(:, 1);
wheel_speed_on   = out_on.state.Data(:, 2);
dist_on          = out_on.state.Data(:, 3);
vehicle_speed_on = out_on.state.Data(:, 4);

%% 3. Plot the Comparisons
figure('Name', 'ABS Performance Comparison', 'Position', [100, 100, 800, 600]);

% Slip Ratio
subplot(2,2,1);
plot(time_off, slip_off, 'r', time_on, slip_on, 'g', 'LineWidth',2);
title('Slip Ratio'); xlabel('Time (s)'); ylabel('Unitless');
legend('ABS Off', 'ABS On'); grid on;

% Stop Distance
subplot(2,2,2);
plot(time_off, dist_off, 'r', time_on, dist_on, 'g', 'LineWidth',2);
title('Stop Distance'); xlabel('Time (s)'); ylabel('Distance (m)');
legend('ABS Off', 'ABS On', 'Location', 'best'); grid on;

% Wheel Speed
subplot(2,2,3);
plot(time_off, wheel_speed_off, 'r', time_on, wheel_speed_on, 'g', 'LineWidth',2);
title('Wheel Speed'); xlabel('Time (s)'); ylabel('rad/s');
legend('ABS Off', 'ABS On', 'Location', 'best'); grid on;

% Vehicle Speed
subplot(2,2,4);
plot(time_off, vehicle_speed_off, 'r', time_on, vehicle_speed_on, 'g', 'LineWidth',2);
title('Vehicle Speed'); xlabel('Time (s)'); ylabel('m/s');
legend('ABS Off', 'ABS On', 'Location', 'best'); grid on;