clc
clear
close all

global L1 L2
L1 = 1;
L2 = 1;
 
q_bef = [0; 0];
qd0 = [0; 0];

% Rectangle
x0_rec = 1; % Center X-coordinate of rectangle
y0_rec = 0.5; % Center Y-coordinate of rectangle
w_rec = 1;  % Width of rectangle
h_rec = 1;  % Height of rectangle

x0 = x0_rec - w_rec / 2;
xf = x0_rec + w_rec / 2;
y0 = y0_rec - h_rec / 2;
yf = y0_rec + h_rec / 2;

t0 = 0;
tf = 1 + 0.25;
delta = 0.008;

figure;
hold on;
axis equal;
grid on;
xlim([-1.5, 2.5]);
ylim([-1.5, 2.5]);

% Plot desired rectangle path as a dotted line
plot([x0 xf xf x0 x0], [y0 y0 yf yf y0], 'k--');

Kp = 10; % gain stability

path_x = [];
path_y = [];

for t = t0:delta:tf
    % path rectangle
    ts = (tf - t0) / 5;
    if t >= t0 && t < t0 + ts
        % First side: from (x0,y0) to (xf,y0)
        xd = x0 + (xf - x0) * (t - t0) / ts;
        yd = y0;
        xdotd = (xf - x0) / ts;
        ydotd = 0;
    elseif t >= t0 + ts && t < t0 + 2*ts
        % Second side: from (xf,y0) to (xf,yf)
        xd = xf;
        yd = y0 + (yf - y0) * (t - (t0 + ts)) / ts;
        xdotd = 0;
        ydotd = (yf - y0) / ts;
    elseif t >= t0 + 2*ts && t < t0 + 3*ts
        % Third side: from (xf,yf) to (x0,yf)
        xd = xf - (xf - x0) * (t - (t0 + 2*ts)) / ts;
        yd = yf;
        xdotd = -(xf - x0) / ts;
        ydotd = 0;
    elseif t >= t0 + 3*ts && t < t0 + 4*ts
        % Fourth side: from (x0,yf) to (x0,y0)
        xd = x0;
        yd = yf - (yf - y0) * (t - (t0 + 3*ts)) / ts;
        xdotd = 0;
        ydotd = -(yf - y0) / ts;
    elseif t >= t0 + 4*ts && t <= tf
        % back to first
        xd = x0 + (xf - x0) * (t - (t0 + 4*ts)) / ts;
        yd = y0;
        xdotd = (xf - x0) / ts;
        ydotd = 0;
    end

    % forward kinematics
    [p0, p1, p2] = ForKin(q_bef);

    % position error
    e = [xd; yd; 0] - p2;

    % desired velocity
    rd = [xdotd; ydotd; 0] + Kp * e;

    % joint velocities
    qd = InvKin(q_bef, rd, qd0);

    % Update joint positions
    q = q_bef + qd * delta;

    % forward kinematics
    [p0, p1, p2] = ForKin(q);
    
    path_x = [path_x; p2(1)];
    path_y = [path_y; p2(2)];

    % Plot the manipulator
    clf; % Clear figure
    hold on;
    axis equal;
    grid on;
    xlim([-1.5, 2.5]);
    ylim([-1.5, 2.5]);
    plot([x0 xf xf x0 x0], [y0 y0 yf yf y0], 'k--'); % Desired path
    plot(path_x, path_y, 'r-', 'LineWidth', 1); % Actual end-effector path
    plot([p0(1) p1(1)], [p0(2) p1(2)], 'b-', 'LineWidth', 2); % Link 1
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'b-', 'LineWidth', 2); % Link 2
    
    % Plot joints
    plot(p0(1), p0(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Base joint
    plot(p1(1), p1(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Joint 1
    plot(p2(1), p2(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k'); % Joint 2
        
    title('2-Link Manipulator Animation');
    xlabel('X Position');
    ylabel('Y Position');
    drawnow;

    q_bef = q;

end
