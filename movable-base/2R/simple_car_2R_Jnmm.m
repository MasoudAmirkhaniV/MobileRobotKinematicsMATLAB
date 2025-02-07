clc
clear
close all

% Manipulator parameters
L1 = 1;
L2 = 1;

q_bef = [0; 0; 0; 0];
theta_bef = 0;
qd0 = [0; 0; 0; 0];

% Circle parameters
xcc = 0; % Center X-coordinate of the circle
ycc = 0;   % Center Y-coordinate of the circle
r = 2;    % Radius of the circle

% Plot desired circular path as a dotted line
theta_circle = linspace(0, 2*pi, 100);
x_circle = xcc + r * cos(theta_circle);
y_circle = ycc + r * sin(theta_circle);
plot(x_circle, y_circle, 'k--');
axis([-5 5 -5 5])
hold on;

% Control gains
K = 70;     % Gain

te = 7;    % Total time
dt = 0.01; % Time step
t = 0:dt:te;

path = zeros(length(t), 2);

path_x = [];
path_y = [];

% Rectangle (car) dimensions
car_length = 1;   % Length of the car
car_width = 0.5;  % Width of the car
half_length = car_length / 2;
half_width = car_width / 2;

for i = 1:length(t)
    x = q_bef(3);
    y = q_bef(4);
    theta = theta_bef;

    xd = xcc + r * cos(t(i));
    yd = ycc + r * sin(t(i));
    xdotd = -r * sin(t(i));
    ydotd = r * cos(t(i));

    p0 = [x; y; 0];
    [p1, p2] = ForKin(p0, q_bef);

    % Jacobian car
    Jb = [cos(theta), 0;
         sin(theta), 0;
         0,          1];
    Jm = Jacob(q_bef);

    JNMM = [Jm Jb];

    e2 = [xd; yd; 0] - p0;
    e_angle = atan2(e2(2), e2(1)) - theta; % Angle to goal

    e = [xd; yd] - p2(1:2);

    rd = [xdotd; ydotd; 0] + K*[e;e_angle];

    qd = pinv(JNMM)*rd + (eye(4) - pinv(JNMM)*JNMM)*qd0;

    % Update joint positions
    q = q_bef + qd * dt;

    % forward kinematics
    [p1, p2] = ForKin(p0, q);

    tt = Jb*q(3:4);
    theta_bef = theta_bef + tt(3)*dt;

    path_x = [path_x; p2(1)];
    path_y = [path_y; p2(2)];

    % Record path
    path(i, :) = q_bef(3:4)';

    % Plot the path up to the current position
    clf; % Clear figure
    hold on;
    grid on;
    axis([-5 5 -5 5])
    plot(x_circle, y_circle, 'k--'); % Desired path (circle)
    plot(path_x, path_y, 'r-', 'LineWidth', 1); % Actual end-effector path

    % Plot the manipulator links
    plot([p0(1) p1(1)], [p0(2) p1(2)], 'b-', 'LineWidth', 2); % Link 1
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'b-', 'LineWidth', 2); % Link 2

    % Plot joints
    plot(p0(1), p0(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Base joint
    plot(p1(1), p1(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Joint 1
    plot(p2(1), p2(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Joint 2

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

    title('2-Link Manipulator Moving Along a Circle');
    xlabel('X Position');
    ylabel('Y Position');
    drawnow;

    q_bef = q;

end
