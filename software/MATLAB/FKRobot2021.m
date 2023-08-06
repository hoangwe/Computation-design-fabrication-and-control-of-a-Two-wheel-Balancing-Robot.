function [T,R,invR,P] = FKRobot2021( a,alpha,d,theta )
syms m1 m2 m3 m4 m5 m6 g e1 e2 e3 de1 de2 de3 dde1 dde2 dde3 l1 l2 l3 
T=simplify([cos(theta)            -sin(theta)           0          a;
            sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
            sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha)  cos(alpha)*d;
            0                     0                     0            1]);
R=simplify(T(1:3,1:3));
invR=simplify(R.');
P=simplify(T(1:3,4));
end
