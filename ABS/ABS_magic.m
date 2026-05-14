model_name = 'ABS'; 
load_system(model_name); 
mdlWks = get_param(model_name, 'ModelWorkspace');

%% 0. RESET: Inject Original Tire Model Data
disp('Resetting to Original Tire Model...');
slip_orig = 0:0.05:1.0; % Original slip array [cite: 139]
mu_orig = [0 0.4 0.8 0.97 1.0 0.98 0.96 0.94 0.92 0.9 0.88 0.855 0.83 0.81 0.79 0.77 0.75 0.73 0.72 0.71 0.7]; % Original mu array [cite: 140, 141]

mdlWks.assignin('slip', slip_orig);
mdlWks.assignin('mu', mu_orig);

%% 1. Run Simulation: Original Tire Model (ABS ON)
disp('Running Original Simulation...');
mdlWks.assignin('ABS_mode', 1); % Ensure ABS is ON
out_old = sim(model_name);

time_old = out_old.tout;
dist_old = out_old.state.Data(:, 3); % Assuming distance is column 3

%% 2. Load and Apply the Magic Formula
disp('Loading Magic Formula...');
run('tire_model.m'); % Runs your script to get slip_ratio and mu

% Push the new Magic Formula variables into the Simulink Model Workspace
mdlWks.assignin('slip', slip_ratio); 
mdlWks.assignin('mu', mu);

%% 3. Run Simulation: Magic Formula Tire Model (ABS ON)
disp('Running Magic Formula Simulation...');
out_new = sim(model_name);

time_new = out_new.tout;
dist_new = out_new.state.Data(:, 3);

%% 4. Plot the Comparisons for your PDF Report
figure('Name', 'Assignment Part 2', 'Position', [100, 100, 800, 400]);

% Plot 1: Tire Curve Comparison 
subplot(1,2,1);
plot(slip_orig, mu_orig, 'r--', 'LineWidth', 2); hold on;
plot(slip_ratio, mu, 'b-', 'LineWidth', 2);
title('Tire Model Comparison');
xlabel('Slip Ratio'); ylabel('Friction Coefficient (\mu)');
legend('Original Lookup Table', 'Magic Formula');
grid on;

% Plot 2: Stopping Distance Comparison
subplot(1,2,2);
plot(time_old, dist_old, 'r--', time_new, dist_new, 'b-', 'LineWidth', 2);
title('Stopping Distance Comparison (ABS ON)');
xlabel('Time (s)'); ylabel('Distance (m)');
legend('Original Tire Model', 'Magic Formula Tire', 'Location', 'best');
grid on;