function out=Draw_robot(the1,the2,the3,m)
    e1=the1;
    e2=the2;
    e3=the3;
    l1=1;l2=1;l3=1;l4=1;

    x=[ 0, 0, l2*cos(e1)*cos(e2), cos(e1)*(l3*cos(e2 + e3) + l2*cos(e2))];
    y=[ 0, 0, l2*cos(e2)*sin(e1), sin(e1)*(l3*cos(e2 + e3) + l2*cos(e2))];
    z=[ -l1, 0,         l2*sin(e2),           l3*sin(e2 + e3) + l2*sin(e2)];
    subplot(2,2,m);
    plot3(x(:),y(:),z(:),'-ro','linewidth',3);
    hold on;
    plot3(x(1,4),y(1,4),z(1,4),'-ko','MarkerFaceColor','k');
    hold on;
    text(x(1,4),y(1,4),z(1,4),['\fontsize{7}','  PEE:(', num2str(x(1,4),1.5), ', ', num2str(y(1,4),1.5),', ', num2str(z(1,4),1.5),')']);
    hold on;
    xlim([-2 2]);
    ylim([-2 2]);
    zlim([-2 2]);
    grid on;
    xlabel('X');ylabel('Y');zlabel('Z');
    title(['\fontsize{8}',sprintf(' Solution: %d CoR with  e1 = %.2f, e2 = %.2f, e3 = %.2f ',m,e1*180/pi,e2*180/pi,e3*180/pi)]);
    pause(0.000001);

end