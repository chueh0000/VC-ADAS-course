% Define the Transport Delay values you want to test (in seconds)
% Example: 0s (ideal), 0.02s (slight delay), 0.05s (bad delay), 0.08s (likely unstable)
T_delay_values = [0, 0.02, 0.04, 0.05]; 
model_name = 'Pure_Pursuit'; % Make sure this matches your model name

% Load the model and get the Model Workspace
load_system(model_name); 
mdlWks = get_param(model_name, 'ModelWorkspace');

% Generate a custom color array so each line is distinct
color_array = lines(length(T_delay_values)); 

% Create a single, clean figure window
figure('Name', 'Comparison on Feedback Delay', 'NumberTitle', 'off', 'Position', [200, 200, 700, 500]);
hold on;

for i = 1:length(T_delay_values)
    current_delay = T_delay_values(i); 
    current_color = color_array(i, :); 
    
    % Update the T_delay variable in the Model Workspace
    mdlWks.assignin('T_delay', current_delay);
    
    % Force Simulink to log states and run
    set_param(model_name, 'SaveState', 'on', 'StateSaveName', 'xout', 'SaveFormat', 'Array');
    simOut = sim(model_name); 
    
    % Extract data
    time = simOut.tout;
    all_states = simOut.xout;
    
    % We only plot State 1 (y). 
    % We also make the line for 0 delay thicker as our "baseline"
    if current_delay == 0
        line_width = 2.5;
    else
        line_width = 1.5;
    end
    
    plot(time, all_states(:, 1), 'Color', current_color, 'LineWidth', line_width, ...
         'DisplayName', ['Delay = ', num2str(current_delay), ' s']);
end

% Formatting the plot
title('Comparison on Feedback Delay');
xlabel('Time (s)', 'FontSize', 12);
ylabel('Magnitude ($y$)', 'Interpreter', 'latex', 'FontSize', 12);
legend('show', 'Location', 'best', 'FontSize', 11);
grid on;
hold off;
