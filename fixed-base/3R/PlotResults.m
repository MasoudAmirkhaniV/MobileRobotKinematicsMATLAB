function PlotResults(t_values, q_values, qd_values, desired_x, end_effector_x)
    % Number of joints
    n = size(q_values, 1);

    %% Figure 1: Joint Positions and Velocities
    figure('Name', 'Joint Positions and Velocities', 'NumberTitle', 'off');
    
    % Adjust layout spacing
    tiledlayout(n, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    for i = 1:n
        % Joint positions
        nexttile;
        plot(t_values, q_values(i, :), 'LineWidth', 1.5);
        xlabel('Time [s]');
        ylabel(sprintf('q_%d [rad]', i));
        title(sprintf('Joint %d Position', i));
        grid on;

        % Joint velocities
        nexttile;
        plot(t_values, qd_values(i, :), 'LineWidth', 1.5);
        xlabel('Time [s]');
        ylabel(sprintf('q_%d dot [rad/s]', i));
        title(sprintf('Joint %d Velocity', i));
        grid on;
    end

    sgtitle('Joint Positions and Velocities');

    %% Figure 2: Desired Path
    figure('Name', 'Desired vs Actual End-Effector Path', 'NumberTitle', 'off');
    hold on;
    axis equal;
    grid on;
    xlabel('X Position');
    ylabel('Y Position');
    title('Desired Path vs End-Effector Path');

    % Plot desired path
    plot(desired_x(1, :), desired_x(2, :), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Desired Path');

    % Plot end-effector path
    plot(end_effector_x(1, :), end_effector_x(2, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'End-Effector Path');

    legend('show');

    %% Figure 3: Desired Path base on time
    figure('Name', 'Desired vs Actual End-Effector Path', 'NumberTitle', 'off');
    hold on;
    axis equal;
    grid on;
    xlabel('Time (s)');
    ylabel('X Position');
    title('Desired X base on time');

    % Plot desired path
    plot(t_values, desired_x(1, :), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Desired Path');

    % Plot end-effector path
    plot(t_values, end_effector_x(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'End-Effector Path');

    legend('show');

    %% Figure 4: Desired Path base on time
    figure('Name', 'Desired vs Actual End-Effector Path', 'NumberTitle', 'off');
    hold on;
    axis equal;
    grid on;
    xlabel('Time (s)');
    ylabel('Y Position');
    title('Desired Y base on time');

    % Plot desired path
    plot(t_values, desired_x(2, :), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Desired Path');

    % Plot end-effector path
    plot(t_values, end_effector_x(2, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'End-Effector Path');

    legend('show');
end
