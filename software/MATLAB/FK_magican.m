function [x,y,z]=FK_magican(the1,the2,the3)
%delclare the parameter:
e1=the1;
e2=the2;
e3=the3;
l1=1;l2=1;l3=1;l4=1;
% End-Effector Position of Robot
 x=cos(e1)*(l3*cos(e2 + e3) + l2*cos(e2));
 y=sin(e1)*(l3*cos(e2 + e3) + l2*cos(e2));
 z=          l3*sin(e2 + e3) + l2*sin(e2);
end