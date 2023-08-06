function out=animate(x,y,z,width,height)
w=width;
h=height;
filename = 'animation_demo.gif'
h = figure('rend','painters','pos',[100 100 w h]); clf;     % Create the figure
set(gcf, 'Color', [1,1,1]);                                 % White backgrounds are nice
hold on
grid on;
%%% Prepare Plot Handles %%%
h_X = plot3(x(:),y(:),z(:),'-ro','linewidth',3); % Plot handle 
axis([min(x), max(x), min(y), max(y),min(z), max(z)]);                     % Set the axis limits so all the data shows up
xlabel ('X'); ylabel('Y'); zlabel('Z');
for n = 1:numel(x),
%%%Step through x and update animation%%%
    h_X.XData = [h_X.XData, x(n)];                          % update plot x and y data
    h_X.YData = [h_X.YData, y(n)];
    h_X.ZData = [h_X.ZData, z(n)];
    drawnow;                                                % draw to the screen
    frame = getframe(h);                                    % capture frame for file-writing
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
     %%% Write the animated GIF %%%
    if n==1,
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf, 'DelayTime', .05); 
    elseif (mod(n, 1))==0,
          imwrite(imind,cm,filename,'gif','WriteMode','append', 'DelayTime', .05); 
     end 
    pause(.02)

end
