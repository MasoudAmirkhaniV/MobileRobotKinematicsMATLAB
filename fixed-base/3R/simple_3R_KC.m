clc
clear
close all

global L1 L2 L3
L1 = 1;
L2 = 1;
L3 = 1;
 
q_bef = [0; 0; pi/2];
qd0 = [0; 0; 0];

xc = 2; % Center X-coordinate of the circle
yc = 0;   % Center Y-coordinate of the circle
r = 0.5;    % Radius of the circle

t0 = 0;
tf = 2*pi; 
delta = 0.01;
tt = t0:delta:tf;
num_steps = length(tt);


figure;
hold on;
axis equal;
grid on;
xlim([-1.5, 2.5]);
ylim([-1.5, 2.5]);

% Plot desired circular path
theta = linspace(0, 2*pi, 100);
x_circle = xc + r * cos(theta);
y_circle = yc + r * sin(theta);
plot(x_circle, y_circle, 'k--');

itter = 0;
q_values = zeros(length(q_bef), num_steps);
qd_values = zeros(length(q_bef), num_steps);
desired_x = zeros(2, num_steps);
end_effector_x = zeros(2, num_steps);

Kp = 10; % gain stability

path_x = [];
path_y = [];

for t = t0:delta:tf

    itter = itter + 1;

    xd = xc + r * cos(t);
    yd = yc + r * sin(t);
    xdotd = -r * sin(t);
    ydotd = r * cos(t);

    desired_x(:, itter) = [xd; yd];


    % forward kinematics
    [p0, p1, p2, p3] = ForKin(q_bef);

    % position error
    e = [xd; yd; 0] - p3;

    % desired velocity
    rd = [xdotd; ydotd; 0] + Kp*e;

    % joint velocities
    qd = InvKin(q_bef, rd, qd0);

    % Update joint positions
    q = q_bef + qd * delta;

    % Store joint positions and velocities
    q_values(:, itter) = q;
    qd_values(:, itter) = qd;


    % forward kinematics
    [p0, p1, p2, p3] = ForKin(q);
    
    % Store end-effector position for plotting path
    path_x = [path_x; p3(1)];
    path_y = [path_y; p3(2)];

    % Store end-effector position
    end_effector_x(:, itter) = p3(1:2);


    q_bef = q;

    % Plot the manipulator
    clf;
    hold on;
    axis equal;
    grid on;
    xlim([-1.5, 2.5]);
    ylim([-1.5, 2.5]);
    plot(x_circle, y_circle, 'k--'); % Desired path (circle)
    plot(path_x, path_y, 'r-', 'LineWidth', 1); % Actual end-effector path
    plot([p0(1) p1(1)], [p0(2) p1(2)], 'b-', 'LineWidth', 2); % Link 1
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'b-', 'LineWidth', 2); % Link 2
    plot([p2(1) p3(1)], [p2(2) p3(2)], 'b-', 'LineWidth', 2); % Link 3
    
    % Plot joints
    plot(p0(1), p0(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Base joint
    plot(p1(1), p1(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Joint 1
    plot(p2(1), p2(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Joint 2
    plot(p3(1), p3(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % End effector
        
    title('3-Link Manipulator Moving Along a Circle');
    xlabel('X Position');
    ylabel('Y Position');
    drawnow;

end

PlotResults(tt, q_values, qd_values, desired_x, end_effector_x);