function out=Draw_robot_v2(the1,the2,the3)
    e1=the1;
    e2=the2;
    e3=the3;
    l1=1;l2=1;l3=1;l4=1;

    x=[ 0, 0, l2*cos(e1)*cos(e2), cos(e1)*(l3*cos(e2 + e3) + l2*cos(e2))];
    y=[ 0, 0, l2*cos(e2)*sin(e1), sin(e1)*(l3*cos(e2 + e3) + l2*cos(e2))];
    z=[ -l1, 0,          -l2*sin(e2),           - l3*sin(e2 + e3) - l2*sin(e2)];
    figure(1);
    plot3(x(:),y(:),z(:),'-ro','linewidth',3);
    hold on;
    plot3(x(1,4),y(1,4),z(1,4),'-ko','MarkerFaceColor','k');
    hold off;
    xlim([-5 5]);
    ylim([-5 5]);
    zlim([-5 5]);
    grid on;
    xlabel('X');ylabel('Y');zlabel('Z');

end