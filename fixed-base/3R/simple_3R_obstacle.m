clc
clear
close all

global L1 L2 L3
L1 = 1;
L2 = 1;
L3 = 1;
 
q_bef = [pi/2; 0; 0];
qd0 = [0; 0; 0];

% Circle parameters
xc = 2; % Center X-coordinate of the circle
yc = 0;   % Center Y-coordinate of the circle
r = 0.5;    % Radius of the circle

% Add obstacle
theta_circle = linspace(0, 2*pi, 100);
obs_x = 1;
obs_y = 0;
obs_radius = 0.2;
obs = [obs_x; obs_y; 0];
fill(obs_x + obs_radius * cos(theta_circle), obs_y + obs_radius * sin(theta_circle), 'black');

t0 = 0;
tf = 2*pi; 
delta = 0.05;

figure;
hold on;
axis equal;
grid on;
xlim([-1.5, 2.5]);
ylim([-1.5, 2.5]);

% Plot desired circular path as a dotted line
theta = linspace(0, 2*pi, 100);
x_circle = xc + r * cos(theta);
y_circle = yc + r * sin(theta);
plot(x_circle, y_circle, 'k--');

Kp = 10; % Proportional gain adjusted for stability

% Initialize arrays to store end-effector path
path_x = [];
path_y = [];

for t = t0:delta:tf
    % desired position circle
    xd = xc + r * cos(t);
    yd = yc + r * sin(t);
    xdotd = -r * sin(t);
    ydotd = r * cos(t);

    % forward kinematics
    [p0, p1, p2, p3] = ForKin(q_bef);

    % position error
    e = [xd; yd; 0] - p3;

    rd = [xdotd; ydotd; 0] + Kp*e;

    n_min = [norm(p1-obs-obs_radius)
             norm(p2-obs-obs_radius)
             norm(p3-obs-obs_radius)];

    qd0 = -10*n_min;

    qd = InvKin(q_bef, rd, qd0);

    % Update joint positions
    q = q_bef + qd * delta;

    % forward kinematics
    [p0, p1, p2, p3] = ForKin(q);
    
    path_x = [path_x; p3(1)];
    path_y = [path_y; p3(2)];

    % Plot the manipulator
    clf; % Clear figure
    hold on;
    axis equal;
    grid on;
    xlim([-1.5, 2.5]);
    ylim([-1.5, 2.5]);
    fill(obs_x + obs_radius * cos(theta_circle), obs_y + obs_radius * sin(theta_circle), 'black');
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

    q_bef = q;

end