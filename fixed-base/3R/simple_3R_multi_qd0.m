clc
clear
close all

global L1 L2 L3
L1 = 1;
L2 = 1;
L3 = 1;
 
q_bef = [0; 0; 0];
qd0 = [0; 0; 0];

% Circle
xc = 1; % Center X-coordinate of the circle
yc = 1;   % Center Y-coordinate of the circle
r = 1;    % Radius of circle

% Define parameters for p2's desired circular path
xc2 = 1;    % Center X-coordinate of circle  2
yc2 = 0;      % Center Y-coordinate of circle 2
r2 = 1;     % Radius of circle 2

t0 = 0;
tf = 2*pi; 
delta = 0.01;

figure;
hold on;
axis equal;
grid on;
xlim([-1.5, 2.5]);
ylim([-1.5, 2.5]);

theta = linspace(0, 2*pi, 100);
x_circle = xc + r * cos(theta);
y_circle = yc + r * sin(theta);
plot(x_circle, y_circle, 'k--');

Kp = 10; % gain stability

path_x = [];
path_y = [];

path2_x = [];
path2_y = [];

for t = t0:delta:tf
    % desired position circle
    xd = xc + r * cos(t);
    yd = yc + r * sin(t);
    xdotd = -r * sin(t);
    ydotd = r * cos(t);

    % forward kinematics
    [p0, p1, p2, p3] = ForKin(q_bef);

    % position circle 2
    xd2 = xc2 + r2 * cos(t);
    yd2 = yc2 + r2 * sin(t);

    % velocity circle 2
    xdotd2 = -r2 * sin(t);
    ydotd2 = r2 * cos(t);


    % position error
    e = [xd; yd; 0] - p3;
    e2 = [xd2; yd2; 0] - p2;


    rd = [xdotd; ydotd; 0] + Kp*e;
    rd2 = [xdotd2; ydotd2; 0] + Kp*e2;
    
    % qd0 second task
    qd0 = pinv(Jacob2(q_bef)*(eye(3) - pinv(Jacob(q_bef))*Jacob(q_bef)))*(rd2 - Jacob2(q_bef)*pinv(Jacob(q_bef))*rd);

    qd = InvKin(q_bef, rd, qd0);

    % Update joint positions
    q = q_bef + qd * delta;

    % forward kinematics
    [p0, p1, p2, p3] = ForKin(q);
    
    % Store end-effector position for plotting path
    path_x = [path_x; p3(1)];
    path_y = [path_y; p3(2)];

    path2_x = [path2_x; p2(1)];
    path2_y = [path2_y; p2(2)];

    % Plot the manipulator
    clf; % Clear figure
    hold on;
    axis equal;
    grid on;
    xlim([-1.5, 2.5]);
    ylim([-1.5, 2.5]);
    plot(x_circle, y_circle, 'k--'); % Desired path (circle)
    plot(path_x, path_y, 'r-', 'LineWidth', 1); % Actual end-effector path
    plot(path2_x, path2_y, 'r-', 'LineWidth', 1); % Actual end-effector path
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