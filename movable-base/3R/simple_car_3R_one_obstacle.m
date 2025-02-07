clc
clear
close all

% Manipulator parameters
L1 = 1;
L2 = 1;
L3 = 1;
q_bef = [0; 0; 0; 0; 0];
theta_bef = 0;
qd0 = [0; 0; 0; 0; 0];

xcc = 2; % Center X-coordinate of the circle
ycc = 0; % Center Y-coordinate of the circle
r = 1; % Radius of the circle

theta_circle = linspace(0, 2*pi, 100);
x_circle = xcc + r * cos(theta_circle);
y_circle = ycc + r * sin(theta_circle);
plot(x_circle, y_circle, 'k--');
axis([-5 5 -5 5])
hold on;

% Add obstacle
obs_x = 0;
obs_y = 1;
obs_radius = 0.5;
obs = [obs_x; obs_y; 0];
fill(obs_x + obs_radius * cos(theta_circle), obs_y + obs_radius * sin(theta_circle), 'black');

K = 70; % Gain

te = 8;    %Total time
dt = 0.01; % Time step
t = 0:dt:te;

path = zeros(length(t), 2);

path_x = [];
path_y = [];
path_q = []; 
path_qd = [];

% Rectangle (car) dimensions
car_length = 1; % Length of the car
car_width = 0.5; % Width of the car
half_length = car_length / 2;
half_width = car_width / 2;

for i = 1:length(t)
    x = q_bef(4);
    y = q_bef(5);
    theta = theta_bef;

    xd = xcc + r * cos(t(i));
    yd = ycc + r * sin(t(i));
    xdotd = -r * sin(t(i));
    ydotd = r * cos(t(i));

    p0 = [x; y; 0];
    [p1, p2, p3] = ForKin(p0, q_bef);

    n_min = [norm(x-obs-obs_radius)
             norm(y-obs-obs_radius)
             norm(p1-obs-obs_radius)
             norm(p2-obs-obs_radius)
             norm(p3-obs-obs_radius)];

    qd0 = -10*n_min;

    % Jacobian car
    Jb = [cos(theta), 0;
         sin(theta), 0;
         0,          1];
    Jm = Jacob(q_bef);

    JNMM = [Jm Jb];

    e2 = [xd; yd; 0] - p0;
    e_angle = atan2(e2(2), e2(1)) - theta; % Angle to goal

    e = [xd; yd] - p3(1:2);

    rd = [xdotd; ydotd; 0] + K*[e;e_angle];

    qd = pinv(JNMM)*rd + (eye(5) - pinv(JNMM)*JNMM)*qd0;

    q = q_bef + qd * dt;

    path_q = [path_q; q'];
    path_qd = [path_qd; qd'];

    % Compute forward kinematics to get joint positions
    [p1, p2, p3] = ForKin(p0, q);

    tt = Jb*q(4:5);
    theta_bef = theta_bef + tt(3)*dt;

    path_x = [path_x; p3(1)];
    path_y = [path_y; p3(2)];

    % Record path
    path(i, :) = q_bef(4:5)';

    clf; % Clear figure
    hold on;
    grid on;
    axis([-5 5 -5 5])
    plot(x_circle, y_circle, 'k--'); % Desired path (circle)
    plot(path_x, path_y, 'r-', 'LineWidth', 1); % Actual end-effector path

    % Plot obstacle
    fill(obs_x + obs_radius * cos(theta_circle), obs_y + obs_radius * sin(theta_circle), 'black');

    % Plot the manipulator links
    plot([p0(1) p1(1)], [p0(2) p1(2)], 'b-', 'LineWidth', 2); % Link 1
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'b-', 'LineWidth', 2); % Link 2
    plot([p2(1) p3(1)], [p2(2) p3(2)], 'b-', 'LineWidth', 2); % Link 3

    % Plot joints
    plot(p0(1), p0(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Base joint
    plot(p1(1), p1(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Joint 1
    plot(p2(1), p2(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Joint 2
    plot(p3(1), p3(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % End effector

    % Define rectangle corners before rotation (relative to center)
    rect_corners = [-half_length, -half_width;
                    -half_length,  half_width;
                     half_length,  half_width;
                     half_length, -half_width;
                    -half_length, -half_width]; % To close the rectangle

    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];

    % Rotate and translate rectangle to the position p0
    rect_rotated = (R * rect_corners')';
    rect_translated = rect_rotated + [p0(1), p0(2)];

    % Plot the rectangle (car)
    fill(rect_translated(:,1), rect_translated(:,2), 'g', 'FaceAlpha', 0.5);
    plot(rect_translated(:,1), rect_translated(:,2), 'k-', 'LineWidth', 2);

    title('3-Link Manipulator Moving Along a Circle with Obstacle');
    xlabel('X Position');
    ylabel('Y Position');
    drawnow;

    q_bef = q;
end

desired_path_x = xcc + r * cos(t); % X-coordinates of desired path
desired_path_y = ycc + r * sin(t); % Y-coordinates of desired path

figure;
subplot(3,1,1);
plot(t, path_q(:, 1), 'r', t, path_q(:, 2), 'g', t, path_q(:, 3), 'b');
legend('q1', 'q2', 'q3');
xlabel('Time (s)');
ylabel('Joint Positions');
title('Joint Position Paths');

subplot(3,1,2);
plot(t, path_qd(:, 1), 'r', t, path_qd(:, 2), 'g', t, path_qd(:, 3), 'b');
legend('qd1', 'qd2', 'qd3');
xlabel('Time (s)');
ylabel('Joint Velocities');
title('Joint Velocity Paths');

subplot(3,1,3);
plot(t, desired_path_x, 'k--', 'DisplayName', 'Desired Path X');
hold on;
plot(t, desired_path_y, 'k:', 'DisplayName', 'Desired Path Y');
plot(t, path_x, 'r-', 'DisplayName', 'Actual Path X');
plot(t, path_y, 'b-', 'DisplayName', 'Actual Path Y');
legend('Desired Path X', 'Desired Path Y', 'Actual Path X', 'Actual Path Y');
xlabel('Time (s)');
ylabel('Position');
title('Desired vs. Actual End-Effector Path Over Time');
hold off;
