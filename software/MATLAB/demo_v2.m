% 3? ????? ?? 2?? ??? ??

% Parameters ??
L1 = 1;     L2 = 1;         % ??? ??
q1 = pi/4;  q2 = -pi/12;      % xy????? ??
r1 = pi/4;  r2 = -pi/12;      % xy???? z? ????? ??


% ?? 1? ? ?? ?? ?? ? ??? ??
x1 = L1*cos(r1)*cos(q1);   Px1 = [0 x1];      % x??
y1 = L1*cos(r1)*sin(q1);   Py1 = [0 y1];      % y??
z1 = L1*cos((pi/2)-r1);    Pz1 = [0 z1];      % z??

% ?? 2? ? ?? ?? ?? ? ??? ??
x2 = x1 + L2*cos(r1+r2)*cos(q1+q2);     Px2 = [x1 x2];      % x??
y2 = y1 + L2*cos(r1+r2)*sin(q1+q2);     Py2 = [y1 y2];      % y??
z2 = z1 + L2*sin(r1+r2);                Pz2 = [z1 z2];      % z??




% ?????? ? ???? ?? plot ??
Fig = figure('Position', [300 300 600 600], 'Color', [1 1 1])
Axis = axes('parent', Fig);
hold on;
grid on;
axis([-2 2 -2 2 -0 2]);
title('2-Link Arm', 'fontsize', 25);   % ?? ??
xlabel('X', 'fontsize', 15)       % x? ?? ??
ylabel('Y', 'fontsize', 15)       % y? ?? ??
zlabel('Z', 'fontsize', 15)       % z? ?? ??

% ?? ??? ??? plot? ??? ???
p1 = plot3(Px1,Py1,Pz1, '-or','Linewidth', 3);
p2 = plot3(Px2,Py2,Pz2, '-or','Linewidth', 3);

% ??? ? ???? ? ??? ???? ??? ??
plot3([0 x1], [0 0], [0 0], '--k', 'Linewidth', 2);         % ?? -> x? ??
plot3([x1 x1], [0 y1], [0 0], '--k', 'Linewidth', 2);       % x? ?? -> x, y? ??
plot3([x1 x1], [y1 y1], [0 z1], '--k', 'Linewidth', 2);     % x, y? ?? -> x, y, z? ??

% plot3([0 x2], [0 0], [0 0], '--b', 'Linewidth', 2);         % ?? -> x? ??
% plot3([x2 x2], [0 y2], [0 0], '--b', 'Linewidth', 2);       % x? ?? -> x, y? ??
% plot3([x2 x2], [y2 y2], [0 z2], '--b', 'Linewidth', 2);     % x, y? ?? -> x, y, z? ??

set(p1, 'XData', Px1, 'YData', Py1, 'ZData', Pz1)
drawnow