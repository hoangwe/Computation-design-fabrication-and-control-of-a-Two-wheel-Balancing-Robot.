function out = draw_controller(a,b,c)
% a = input signal times
% b = input signal
% c = number of input

lw = 1.5;
ms = 1;
d1 ='-.r';
d2 ='-.k';
d3 ='-.b';
d4 ='--r';
d5 ='--k';
d6 ='--b';


d = [d1;d2;d3;d4;d5;d6];

 for i=1:c,
     plot(a,b(:,i),d(i,1:3),'LineWidth', lw, 'MarkerSize', ms);
     hold on;
     grid on;
 end
 drawnow;
 
end