syms m M R Jw n Jm L n anp bet fw g W J_phi vl vr J_psi x1 x2 x3 x4 x5 x6 x7 x8 x9;


f1 = ((2*m + M)*R^2 + 2*Jw + 2*n^2*Jm)*x3 + (M*L*R*cos(x4) - 2*n^2*Jm)*x6 - M*L*R*x5^2*sin(x4)-(anp*(vl+vr)-2*(bet+fw)*x2+2*bet*x5);
f2 = (M*L*R*cos(x4) - 2*n^2*Jm)*x3 + (M*L^2 + J_psi + 2*n^2*Jm)*x6 - (M*g*L*sin(x4) + M*L^2*x8^2*sin(x4)*cos(x4))-(-anp*(vl+vr)+2*bet*x2-2*bet*x5);
f3 = (0.5*m*W^2 + J_phi + 0.5*W^2*(Jw + n^2*Jm)/R^2 + M*L^2*sin(x4)^2)*x9 + 2*M*L^2*x5*x8*sin(x4)*cos(x4)-(W*anp*(vr-vl)/(2*R)-W^2*((bet+fw)*x8)/2*R^2);

f11=((2*m+M)*R*R+2*Jw+2*n*n*Jm)*x3+(M*R*L*cos(x4)-2*n*n*Jm)*x6-M*L*R*x5*x5*sin(x4)-anp*(vl+vr)+2*(bet+fw)*x2-2*bet*x5 -(((2*m + M)*R^2 + 2*Jw + 2*n^2*Jm)*x3 + (M*L*R*cos(x4) - 2*n^2*Jm)*x6 - M*L*R*x5^2*sin(x4)-(anp*(vl+vr)-2*(bet+fw)*x2+2*bet*x5));
f22=(M*L*R*cos(x4)-2*n*n*Jm)*x3+(M*L*L+J_psi+2*n*n*Jm)*x6-M*g*L*sin(x4)-M*L*L*x8*x8*sin(2*x4)+anp*(vl+vr)-2*bet*x2+2*bet*x5-((M*L*R*cos(x4) - 2*n^2*Jm)*x3 + (M*L^2 + J_psi + 2*n^2*Jm)*x6 - (M*g*L*sin(x4) + M*L^2*x8^2*sin(x4)*cos(x4))-(-anp*(vl+vr)+2*bet*x2-2*bet*x5));
f33=(0.5*m*W*W+J_phi+W*W/2/R^2*(Jw+n*n*Jm)+M*L*L*sin(x4)*sin(x4))*x9+2*M*L*L*x5*x8*sin(x4)*cos(x4)-W/2/R*anp*(vr-vl)+W^2/2/R^2*(bet+fw)*x8-((0.5*m*W^2 + J_phi + 0.5*W^2*(Jw + n^2*Jm)/R^2 + M*L^2*sin(x4)^2)*x9 + 2*M*L^2*x5*x8*sin(x4)*cos(x4)-(W*anp*(vr-vl)/(2*R)-W^2*((bet+fw)*x8)/2*R^2));


f1-f11
f2-f22
f3-f33