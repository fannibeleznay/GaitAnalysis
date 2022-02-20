function waistEstimator_automatic
    close all
    clc
    
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline(); %0.07
    
    % Create an empty Point Cloud from the RealSense
    pointcloud = realsense.pointcloud(); %0.05
    
    % Start streaming on an arbitrary camera with default settings
    config = realsense.config();
    config.enable_stream(realsense.stream.depth,640,360,...
        realsense.format.z16,30);
    config.enable_stream(realsense.stream.color,640,360,...
        realsense.format.rgb8,30) % 0.07
    
    profile = pipe.start(config); %0.65
    
%     color_stream = profile.get_stream(realsense.stream.color);
%     color_video_stream = color_stream.as('video_stream_profile');
%     intr = color_video_stream.get_intrinsics();
%     imagePoints = worldToImage(cameraIntrinsics([intr.fx intr.fy],[intr.ppx intr.ppy],[360 640]),[1 0 0; 0 0 -1; 0 1 0],[0 0 0],ptcl_zone.Location);

%     worldPoints = pointsToWorld(cameraIntrinsics([intr.fxintr.fy],[intr.ppx intr.ppy],[360 640]),[1 0 0; 0 0 1; 0 -1 0],[0 0 0],imagePoints);
    
    
    % transformation
    tform = rigid3d([1 0 0; 0 0 -1; 0 1 0],[0 0 0]); % 0.15
      
    player = pcplayer([-0.25 0.25], [0.1 0.8], [-0.2 0.2]);
    title = player.Axes.Title;
    
    figure(2);
    subplot(3,1,1)
    grid on
    ylim([-0.25 0.25])   
    li_x = animatedline(gca);

    subplot(3,1,2)
    grid on
    ylim([0.1 0.8])
    li_y = animatedline(gca);
    
    subplot(3,1,3)
    grid on
    ylim([-45 45])
    li_th = animatedline(gca);

    fs = pipe.wait_for_frames(); %0.42
        
    %Depth Frame
    depth = fs.get_depth_frame(); %0.005

    flag = 0;
    while flag == 0
     if (depth.logical())
            
            points = pointcloud.calculate(depth); %0.065            
            vertices = points.get_vertices(); %0.001
            ptcl = pointCloud(vertices(rem(1:height(vertices),30)==0,:)); %0.14

            % ptcl = pcdownsample(ptcl,'random',0.2);
            ptcl_out = pctransform(ptcl,tform); %0.06
            indices = findPointsInROI(ptcl_out,[-0.25 0.25 0.1 0.8 -0.15 0.15]); %0.20
            ptcl_zone = select(ptcl_out,indices); %0.08
            ptcl_zone.Color = lab2uint8(repmat([128 128 128],ptcl_zone.Count,1));
            mean_zone = mean(ptcl_zone.Location);
            flag = 1;
            
            range = [mean_zone(2)-0.1 mean_zone(2)+0.2];
            
     end
    end
    
    numFaces = 30;
    [x,y,z] = sphere(numFaces);
            
    % Main loop
    tic
    for i = 1:10000

        % Obtain frames from a streaming device
        fs = pipe.wait_for_frames(); %0.42
        
        % Divide in Depth and Color Fram
        depth = fs.get_depth_frame(); %0.005
        
        % Produce pointcloud
        if depth.logical()
            
            points = pointcloud.calculate(depth); %0.065  
            vertices = points.get_vertices(); %0.001
            ptcl = pointCloud(vertices(rem(1:height(vertices),15)==0,:)); %0.14

            ptcl_out = pctransform(ptcl,tform); %0.06
            indices = findPointsInROI(ptcl_out,[-0.25 0.25 range -0.15 0.15]); %0.20
            ptcl_zone = select(ptcl_out,indices); %0.08
            ptcl_zone.Color = lab2uint8(repmat([128 128 128],ptcl_zone.Count,1));
            mean_zone = mean(ptcl_zone.Location);
            
            indices_base = findPointsInROI(ptcl_zone,[ptcl_zone.XLimits(1)...
                ptcl_zone.XLimits(2) range mean_zone(3)+0.02 mean_zone(3)+0.07]);
            ptcl_base = select(ptcl_zone,indices_base); %0.006
            
            ptcl_zone_mean = pointCloud(([x(:),y(:),z(:)]*0.005)+[mean_zone(1) mean_zone(2) mean_zone(3)]);
            ptcl_zone_mean.Color = lab2uint8(repmat([0 255 0],ptcl_zone_mean.Count,1)); %0.001
            
            indices_left = findPointsInROI(ptcl_zone,[ptcl_base.XLimits(1)...
                ptcl_base.XLimits(1)+0.05 range mean_zone(3)+0.02 mean_zone(3)+0.07]); %0.03
            ptcl_left = select(ptcl_zone,indices_left); %0.006
            ptcl_left.Color = lab2uint8(repmat([255 0 0],ptcl_left.Count,1)); %0.03
            mean_left = mean(ptcl_left.Location); %0.02

            indices_right = findPointsInROI(ptcl_zone,[ptcl_base.XLimits(2)-0.05 ...
                ptcl_base.XLimits(2) range mean_zone(3)+0.02 mean_zone(3)+0.07]); %0.003
            ptcl_right = select(ptcl_zone,indices_right); %0.02
            ptcl_right.Color = lab2uint8(repmat([200 0 200],ptcl_right.Count,1)); %0.002
            mean_right = mean(ptcl_right.Location); %0.0002
            
            mean_lrtotal = (mean_right+mean_left)/2;            
    
            if length(mean_left) == 3 && length(mean_right) == 3 
                angle = real(asind((mean_left(2)-mean_right(2))/(mean_left(1)-mean_right(1)))); %0.0003
                
                ptcl_left_mean = pointCloud(([x(:),y(:),z(:)]*0.005)+[mean_left(1) mean_left(2) mean_left(3)]);
                ptcl_left_mean.Color = lab2uint8(repmat([255 0 0],ptcl_left_mean.Count,1)); %0.001
                
                ptcl_right_mean = pointCloud(([x(:),y(:),z(:)]*0.005)+[mean_right(1) mean_right(2) mean_right(3)]);
                ptcl_right_mean.Color = lab2uint8(repmat([200 0 200],ptcl_right_mean.Count,1)); %0.001
                
                u = (mean_left-mean_right)/norm(mean_left-mean_right); %0.001
                line = mean_right + (0:0.0005:norm(mean_left-mean_right))'*u; %0.001 
                
                ptcl_line = pointCloud(line); %0.004
                ptcl_line.Color = lab2uint8(repmat([255 255 0],ptcl_line.Count,1)); %0.001
                
                ptcl_lrtotal_mean = pointCloud(([x(:),y(:),z(:)]*0.005)+[mean_lrtotal(1) mean_lrtotal(2) mean_lrtotal(3)]);
                ptcl_lrtotal_mean.Color = lab2uint8(repmat([255 255 0],ptcl_lrtotal_mean.Count,1)); %0.001
                
                view(player,pccat([ptcl_zone ptcl_zone_mean ptcl_line ptcl_lrtotal_mean ptcl_right ptcl_right_mean ptcl_left ptcl_left_mean]));
                title.String = num2str(angle); 

                addpoints(li_x,toc,mean_zone(1));
                drawnow limitrate
                subplot(3,1,1)
                subtitle(['X position = ' num2str(mean_zone(1))])
                xlim([toc-10 toc+10]) 
                ylabel('x [m]')
                

                addpoints(li_y,toc,mean_zone(2)); 
                drawnow limitrate
                subplot(3,1,2)
                subtitle(['Y position = ' num2str(mean_zone(2))])
                xlim([toc-10 toc+10])
                ylabel('y [m]')

                
                addpoints(li_th,toc,angle);
                drawnow limitrate
                subplot(3,1,3)
                subtitle('Angle position')
                subtitle(['Angle position = ' num2str(angle)])
                xlim([toc-10 toc+10])  
                xlabel('t [s]')
                ylabel('\theta [°]')

                range = [mean_zone(2)-0.15 mean_zone(2)+0.15];
            
            end                      
        end    
    end
end