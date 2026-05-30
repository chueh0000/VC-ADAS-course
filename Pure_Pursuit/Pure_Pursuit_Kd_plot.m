% Define the three Kd values to test
Kd_test_values = [0.23, 0.33, 0.43]; 
model_name = 'Pure_Pursuit';

% Load the model and get the Model Workspace
load_system(model_name); 
mdlWks = get_param(model_name, 'ModelWorkspace');

% --- NEW: Generate a custom color array ---
% lines(N) creates an N-by-3 matrix of the default MATLAB RGB colors
color_array = lines(length(Kd_test_values)); 

% Create a larger figure window for the 5-panel layout
figure('Name', 'Comparison between Kd values', 'NumberTitle', 'off', 'Position', [100, 100, 900, 700]);

for i = 1:length(Kd_test_values)
    current_Kd = Kd_test_values(i); 
    
    % --- NEW: Grab the specific color for this loop iteration ---
    current_color = color_array(i, :); 
    
    % Update Kd in the Model Workspace
    mdlWks.assignin('Kd', current_Kd);
    
    % Force Simulink to log states as an Array
    set_param(model_name, 'SaveState', 'on', 'StateSaveName', 'xout', 'SaveFormat', 'Array');
    
    % Run the simulation
    simOut = sim(model_name); 
    
    % Extract time, states, and our new reference signal
    time = simOut.tout;
    all_states = simOut.xout;
    ref_signal = simOut.ref_data; 
    
    % =========================================================
    % THE BOTTOM SCOPE: Tracking Performance
    % =========================================================
    subplot(3, 2, [1, 2]); 
    hold on;
    
    if i == 1
        % Keep the reference signal strictly black and dashed
        plot(time, ref_signal, 'w--', 'DisplayName', 'Target Reference', 'LineWidth', 1.5);
    end
    
    % Apply the custom color
    plot(time, all_states(:, 1), 'Color', current_color, 'DisplayName', ['Kd = ', num2str(current_Kd)], 'LineWidth', 1.5);
    title('Comparison between Kd values');
    xlabel('Time (s)');
    ylabel('Magnitude');
    grid on;
    if i == length(Kd_test_values) 
        legend('show', 'Location', 'best');
    end
    
    % =========================================================
    % THE TOP SCOPE: Internal State Dynamics
    % =========================================================
    
    % --- State 1 (x_1) ---
    subplot(3, 2, 3);
    hold on;
    plot(time, all_states(:, 1), 'Color', current_color, 'LineWidth', 1.5);
    title('$y$', 'Interpreter', 'latex');
    grid on;
    
    % --- State 2 (x_2) ---
    subplot(3, 2, 4);
    hold on;
    plot(time, all_states(:, 2), 'Color', current_color, 'LineWidth', 1.5);
    title('$\dot{y}$', 'Interpreter', 'latex');
    grid on;
    
    % --- State 3 (x_3) ---
    subplot(3, 2, 5);
    hold on;
    plot(time, all_states(:, 3), 'Color', current_color, 'LineWidth', 1.5);
    title('$\phi$', 'Interpreter', 'latex');
    xlabel('Time (s)');
    grid on;
    
    % --- State 4 (x_4) ---
    subplot(3, 2, 6);
    hold on;
    plot(time, all_states(:, 4), 'Color', current_color, 'LineWidth', 1.5);
    title('$\dot{\phi}$', 'Interpreter', 'latex');
    xlabel('Time (s)');
    grid on;
end
