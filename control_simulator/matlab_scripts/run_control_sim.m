function result = run_control_sim(algorithm_name, params, sim_time, initial_state, ref_signal, ref_amplitude)
% RUN_CONTROL_SIM Main simulation function for control algorithms
%
% Inputs:
%   algorithm_name - Name of controller ('pid', 'lqr', 'mpc')
%   params - Controller parameters (array)
%   sim_time - Simulation duration (seconds)
%   initial_state - Initial system state (empty for default)
%   ref_signal - Reference signal type ('step', 'ramp', 'sine')
%   ref_amplitude - Amplitude of reference signal
%
% Output:
%   result - Struct with simulation results

    % Time vector
    dt = 0.01;  % 10ms sampling time
    time = 0:dt:sim_time;
    N = length(time);
    
    % System definition (simple second-order mass-spring-damper)
    % State: [position; velocity]
    % x_dot = Ax + Bu
    % y = Cx + Du
    m = 1.0;    % mass (kg)
    b = 0.5;    % damping (N-s/m)
    k = 2.0;    % spring constant (N/m)
    
    A = [0, 1; -k/m, -b/m];
    B = [0; 1/m];
    C = [1, 0];
    D = 0;
    
    % Initial state
    if isempty(initial_state)
        x = [0; 0];  % Start at rest
    else
        x = initial_state(:);
    end
    
    % Generate reference signal
    reference = generate_reference(time, ref_signal, ref_amplitude);
    
    % Pre-allocate arrays
    states = zeros(2, N);
    outputs = zeros(1, N);
    controls = zeros(1, N);
    errors = zeros(1, N);
    
    % Controller state (if needed)
    controller_state = [];
    
    % Simulation loop
    for i = 1:N
        % Store current state
        states(:, i) = x;
        y = C * x + D * 0;
        outputs(i) = y;
        
        % Compute error
        r = reference(i);
        e = r - y;
        errors(i) = e;
        
        % Compute control input based on algorithm
        switch lower(algorithm_name)
            case 'pid'
                [u, controller_state] = pid_controller(e, dt, params, controller_state);
            case 'lqr'
                u = lqr_controller(x, r, params, A, B, C);
            case 'mpc'
                u = mpc_controller(x, r, params, A, B, C, dt);
            otherwise
                error(['Unknown algorithm: ' algorithm_name]);
        end
        
        % Saturate control (actuator limits)
        u_max = 10.0;
        u = max(min(u, u_max), -u_max);
        controls(i) = u;
        
        % System dynamics (Euler integration)
        x_dot = A * x + B * u;
        x = x + x_dot * dt;
    end
    
    % Package results
    result = struct();
    result.time = time;
    result.states = outputs;  % Return output (position) as main state
    result.control = controls;
    result.reference = reference;
    result.error = errors;
    result.full_states = states;  % Include all internal states
end


function ref = generate_reference(time, signal_type, amplitude)
% GENERATE_REFERENCE Create reference signal
    
    switch lower(signal_type)
        case 'step'
            ref = amplitude * ones(size(time));
            
        case 'ramp'
            ref = amplitude * time / max(time);
            
        case 'sine'
            freq = 0.5;  % Hz
            ref = amplitude * sin(2 * pi * freq * time);
            
        case 'square'
            freq = 0.2;  % Hz
            ref = amplitude * square(2 * pi * freq * time);
            
        otherwise
            ref = amplitude * ones(size(time));
    end
end
