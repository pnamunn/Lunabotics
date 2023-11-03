clear; % Clear variables
clc; % Clear command window

% Main loop to allow repeating the whole process
repeatProgram = true;
while repeatProgram
    
    % Input Terminal Voltage for No-Load Test
    V_noLoad = input(['Enter the terminal ' ...
        'voltage used for the no-load test: ']);

    % Ask if a gearbox exists
    gearbox = input(['Is there a gearbox? ' ...
        '(yes = 1, no = 0): ']);

    if gearbox
        % Get gear ratio
        pinion_teeth = input(['Enter' ...
            ' the number of teeth on' ...
            ' the pinion gear: ']);

        output_teeth = input(['Enter' ...
            ' the number of teeth on' ...
            ' the output gear: ']);
        gear_ratio = output_teeth / pinion_teeth;

        % Get observed output shaft frequency
        f_out = input(['Enter the ' ...
            'observed frequency at ' ...
            'the output shaft (in Hz): ']);

        f_rotor = f_out * gear_ratio;

    else
        % Directly get rotor frequency
        f_rotor = input(['Enter the ' ...
            'observed frequency of ' ...
            'the rotor (in Hz): ']);

    end

    % Convert frequency to angular velocity
    w_noLoad = 2 * pi * f_rotor;

    % Calculate k_t
    k_t = V_noLoad / w_noLoad;

    % Input for Stall Test
    V_stallTest = input(['Enter the' ...
        ' terminal voltage used for' ...
        ' the stall test: ']);
    
    I_stallTest = input(['Enter the' ...
        ' observed current during ' ...
        'the stall test (in A): ']);

    % Project the stall current 
    % for the no-load voltage
    I_stallProjected = I_stallTest ...
    * (V_noLoad / V_stallTest);

    % Calculate projected 
    % stall torque
    T_stallProjected = k_t * ...
        I_stallProjected;

    % Display results
    fprintf(['Torque constant, ' ...
        'k_t = %f\n'], k_t);
    fprintf(['Projected stall ' ...
        'current at no-load ' ...
        'voltage = %f A\n'], I_stallProjected);
    fprintf(['Projected stall ' ...
        'torque at no-load voltage = %f Nm\n'], ...
        T_stallProjected);
 fprintf('No load speed is  = %f rad/s\n', w_noLoad);

    % Compute initial speed-torque curve 
    % for the no-load voltage
    T_range = linspace(0, T_stallProjected, 100);
    w_range = w_noLoad - (V_noLoad / k_t) * T_range;
    w_range(w_range < 0) = 0;

    % Find maximum torque and speed for axis limits
    maxTorque = T_stallProjected;
    maxSpeed = w_noLoad;

    figure; % Create a new figure
    plot(T_range, w_range, '-b', 'DisplayName', ...
        sprintf('Rotor (V = %f V)', V_noLoad));

    if gearbox
        T_out_range = T_range * gear_ratio; % Gear magnified Torque
        w_out_range = w_range / gear_ratio; % Gear reduced Speed
        hold on;
        plot(T_out_range, w_out_range, '-r', 'DisplayName', ...
            sprintf('Output Shaft (V = %f V)', V_noLoad));
    end

    hold on;
    title('Speed-Torque Curve');
    xlabel('Torque (Nm)');
    ylabel('Speed (rad/s)');
    legend;

    xlim([0 maxTorque]);
    ylim([0 maxSpeed]);

% Exploration of other voltages
explore = input(['Do you want to explore the' ...
    ' speed-torque curve under different ' ...
    'terminal voltages? (yes = 1, no = 0): ']);

while explore
    V_explore = input(['Enter the terminal ' ...
        'voltage you want to explore: ']);

    I_stallExplore = I_stallProjected * (V_explore / V_noLoad);
    T_stallExplore = k_t * I_stallExplore;
    w_new = w_noLoad*(V_explore/V_noLoad);

    % Compute speed-torque curve for the explored voltage
    T_range_explore = linspace(0, T_stallExplore, 100);
    w_range_explore = w_new - (V_explore / k_t)...
        * T_range_explore;

    % Temp for Explored V
    w_range_explore(w_range_explore < 0) = 0;
    fprintf('Stall torque at %f V is %f Nm\n', ...
        V_explore, T_stallExplore);

    plot(T_range_explore, w_range_explore, ...
        'DisplayName', sprintf(['Rotor ' ...
        '(V = %f V)'], V_explore));
    
    maxTorque = max([maxTorque, T_range_explore(end)]);
    maxSpeed = max([maxSpeed, w_range_explore(1)]); 

    if gearbox
        T_out_range_explore = T_range_explore * gear_ratio;
        w_out_range_explore = w_range_explore / gear_ratio;
        hold on;
        plot(T_out_range_explore, w_out_range_explore, ...
            '--', 'DisplayName', sprintf(['Output' ...
            ' Shaft (V = %f V)'], V_explore));
        
        maxTorque = max([maxTorque, T_out_range_explore(end)]);
    end

    legend;
    xlim([0 maxTorque]);
    ylim([0 maxSpeed]);
    explore = input(['Do you want to explore the ' ...
        'speed-torque curve under another ' ...
        'terminal voltage? (yes = 1, no = 0): ']);
end



    % Ask user if they want to repeat 
    % the program or terminate
    repeatProgram = input(['Do you want to run' ...
        ' the program again? (yes = 1, no = 0): ']);
    if ~repeatProgram
        clear; % Clear variables if the program is terminated
        clc; % Clear command window
    end
end
