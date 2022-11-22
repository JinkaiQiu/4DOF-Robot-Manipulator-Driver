function animation_lyc(joint,path,movie,speed,Mode)
% global c
px = path(1,:);
py = path(2,:);
pz = path(3,:);

% h = 0.2; % base height

% Create Movie
if movie == 1
    if Mode == 1
        v = VideoWriter('test_lyc_mode1.avi');
    elseif Mode == 2
        v = VideoWriter('test_lyc_mode2.avi');
    end
    open(v);
end

for i = 1:speed:length(joint)
    
    [fx,fy,fz,T] = FK_lyc(joint(:,i));
    
    % Base
    plot3([fx(1) fx(2)],[fy(1) fy(2)],[fz(1) fz(2)],'k','linewidth',8);
    hold on;
    % Manipulator
    plot3(fx(2:end-1),fy(2:end-1),fz(2:end-1),'k','linewidth',5);
    % Tool
    plot3(fx(end-1:end),fy(end-1:end),fz(end-1:end),'m','linewidth',3);
    % Frames
    for j = 1:6
        Rj = T{j}(1:3,1:3);
        mag = 0.025;
        plot3(fx(j)+[0 Rj(1,1)]*mag,fy(j)+[0 Rj(2,1)]*mag,fz(j)+[0 Rj(3,1)]*mag,'r','linewidth',2); % x
        plot3(fx(j)+[0 Rj(1,2)]*mag,fy(j)+[0 Rj(2,2)]*mag,fz(j)+[0 Rj(3,2)]*mag,'g','linewidth',2); % y
        plot3(fx(j)+[0 Rj(1,3)]*mag,fy(j)+[0 Rj(2,3)]*mag,fz(j)+[0 Rj(3,3)]*mag,'b','linewidth',2); % z
    end
    % Trajectory
    plot3(px,py,pz,'b');
    % Ground
    X = [1 -1;1 -1]*0.2;
    Y = [1 1;-1 -1]*0.2;
    Z = [1 1;1 1]*0;
    surf(X,Y,Z,'FaceColor',[0.9 0.9 0.9],'edgecolor','none'); hold on;
    % Label
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    axis([-0.2 0.2 -0.2 0.2 0 0.2]);
    pbaspect([1 1 1]); % For equal axis lengths in all directions
    grid on;
%     view(30,40);
    view(40,30);

    hold off;
    drawnow;
    
    if movie == 1
        frame = getframe(gcf);
        writeVideo(v,frame);
    end

end

if movie == 1
    close(v)
end

end