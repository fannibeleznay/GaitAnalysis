function waistEstimator_automatic
    close all
    clc
    
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    
    % Create an empty Point Cloud from the RealSense
    pointcloud = realsense.pointcloud();
    
    % Start streaming on an arbitrary camera with default settings
    config = realsense.config();
    config.enable_stream(realsense.stream.depth,640,360,...
        realsense.format.z16,30);
    config.enable_stream(realsense.stream.color,640,360,...
        realsense.format.rgb8,30)
    
    pipe.start(config);
    
%     profile = pipe.start(config);
%     color_stream = profile.get_stream(realsense.stream.color);
%     color_video_stream = color_stream.as('video_stream_profile');
%     intr = color_video_stream.get_intrinsics();
%     imagePoints = worldToImage(cameraIntrinsics([intr.fx intr.fy],[intr.ppx intr.ppy],[360 640]),[1 0 0; 0 0 -1; 0 1 0],[0 0 0],ptcl_zone.Location);

%     worldPoints = pointsToWorld(cameraIntrinsics([intr.fxintr.fy],[intr.ppx intr.ppy],[360 640]),[1 0 0; 0 0 1; 0 -1 0],[0 0 0],imagePoints);
    
    
    % transformation
    tform = rigid3d([1 0 0; 0 0 -1; 0 1 0],[0 0 0]);
      
    player = pcplayer([-0.25 0.25], [0.1 0.8], [-0.2 0.2]);
    title = player.Axes.Title;
    
    figure(2);
    subplot(4,1,1)
    grid on
    ylim([-0.25 0.25])   
    li_y = animatedline(gca);

    subplot(4,1,2)
    grid on
    ylim([-45 45])
    li_th = animatedline(gca);
    
    subplot(4,1,3)
    grid on
    yline(35,'r')
    ylim([-30 60])
    li_th_r = animatedline(gca);
    
    subplot(4,1,4)
    grid on
    yline(35,'r')
    ylim([-30 60])
    li_th_l = animatedline(gca);

    fs = pipe.wait_for_frames();
        
    %Depth Frame
    depth = fs.get_depth_frame();

    flag = 0;
    while flag == 0
     if (depth.logical())
            
            points = pointcloud.calculate(depth);           
            vertices = points.get_vertices();
            ptcl = pointCloud(vertices(rem(1:height(vertices),30)==0,:));

            ptcl_out = pctransform(ptcl,tform);
            indices = findPointsInROI(ptcl_out,[-0.25 0.25 0.1 0.8 -0.15 0.15]);
            ptcl_zone = select(ptcl_out,indices);
            ptcl_zone.Color = lab2uint8(repmat([128 128 128],ptcl_zone.Count,1));
            mean_zone = mean(ptcl_zone.Location);
            flag = 1;
            
            range = [mean_zone(2)-0.1 mean_zone(2)+0.2];
            
     end
    end
    
    numFaces = 15;
    [x,y,z] = sphere(numFaces);
            
    % Main loop
    tic
    for i = 1:10000

        % Obtain frames from a streaming device
        fs = pipe.wait_for_frames();
        
        % Divide in Depth and Color Fram
        depth = fs.get_depth_frame();
        
        % Produce pointcloud
        if depth.logical()
            
            points = pointcloud.calculate(depth);  
            vertices = points.get_vertices();
            ptcl = pointCloud(vertices(rem(1:height(vertices),15)==0,:));

            ptcl_out = pctransform(ptcl,tform);
            indices = findPointsInROI(ptcl_out,[-0.25 0.25 range -0.15 0.15]);
            ptcl_zone = select(ptcl_out,indices);
            ptcl_zone.Color = lab2uint8(repmat([128 128 128],ptcl_zone.Count,1));
            mean_zone = mean(ptcl_zone.Location);
            
            indices_base = findPointsInROI(ptcl_zone,[ptcl_zone.XLimits(1)...
                ptcl_zone.XLimits(2) range mean_zone(3)+0.02 mean_zone(3)+0.07]);
            ptcl_base = select(ptcl_zone,indices_base);
            
            ptcl_zone_mean = pointCloud(([x(:),y(:),z(:)]*0.005)+[mean_zone(1) mean_zone(2) mean_zone(3)]);
            ptcl_zone_mean.Color = lab2uint8(repmat([0 255 0],ptcl_zone_mean.Count,1)); %green
            
            indices_left = findPointsInROI(ptcl_zone,[ptcl_base.XLimits(1)...
                ptcl_base.XLimits(1)+0.05 range mean_zone(3)+0.02 mean_zone(3)+0.07]);
            ptcl_left = select(ptcl_zone,indices_left);
            ptcl_left.Color = lab2uint8(repmat([255 0 0],ptcl_left.Count,1)); %red
            mean_left = mean(ptcl_left.Location);

            indices_right = findPointsInROI(ptcl_zone,[ptcl_base.XLimits(2)-0.05 ...
                ptcl_base.XLimits(2) range mean_zone(3)+0.02 mean_zone(3)+0.07]);
            ptcl_right = select(ptcl_zone,indices_right);
            ptcl_right.Color = lab2uint8(repmat([200 0 200],ptcl_right.Count,1)); %magenta
            mean_right = mean(ptcl_right.Location);
            
            indices_left_hip = findPointsInROI(ptcl_zone,[ptcl_base.XLimits(1)...
                ptcl_base.XLimits(1)+0.05 range mean_zone(3)-0.15 mean_zone(3)-0.10]);
            ptcl_left_hip = select(ptcl_zone,indices_left_hip);
            ptcl_left_hip.Color = lab2uint8(repmat([0 0 255],ptcl_left_hip.Count,1)); %blue
            mean_left_hip = mean(ptcl_left_hip.Location);
            
            indices_right_hip = findPointsInROI(ptcl_zone,[ptcl_base.XLimits(2)-0.05 ...
                ptcl_base.XLimits(2) range mean_zone(3)-0.15 mean_zone(3)-0.10]);
            ptcl_right_hip = select(ptcl_zone,indices_right_hip);
            ptcl_right_hip.Color = lab2uint8(repmat([0 0 0],ptcl_right_hip.Count,1)); %acqua
            mean_right_hip = mean(ptcl_right_hip.Location);
            
            mean_lrtotal = (mean_right+mean_left)/2;
    
            if length(mean_left) == 3 && length(mean_right) == 3  && length(mean_left_hip) == 3 && length(mean_right_hip) == 3 
                
                ptcl_left_mean = pointCloud(([x(:),y(:),z(:)]*0.005)+[mean_left(1) mean_left(2) mean_left(3)]);
                ptcl_left_mean.Color = lab2uint8(repmat([255 0 0],ptcl_left_mean.Count,1));
                
                ptcl_left_hip_mean = pointCloud(([x(:),y(:),z(:)]*0.005)+[mean_left_hip(1) mean_left_hip(2) mean_left_hip(3)]);
                ptcl_left_hip_mean.Color = lab2uint8(repmat([0 0 255],ptcl_left_mean.Count,1));
                
                ptcl_right_mean = pointCloud(([x(:),y(:),z(:)]*0.005)+[mean_right(1) mean_right(2) mean_right(3)]);
                ptcl_right_mean.Color = lab2uint8(repmat([200 0 200],ptcl_right_mean.Count,1));
                
                ptcl_right_hip_mean = pointCloud(([x(:),y(:),z(:)]*0.005)+[mean_right_hip(1) mean_right_hip(2) mean_right_hip(3)]);
                ptcl_right_hip_mean.Color = lab2uint8(repmat([0 0 0],ptcl_right_mean.Count,1));
                
                u = (mean_left-mean_right)/norm(mean_left-mean_right);
                line = mean_right + (0:0.0005:norm(mean_left-mean_right))'*u; 

                
                u_lhip = (mean_left-mean_left_hip)/norm(mean_left-mean_left_hip);
                line_l = mean_left_hip + (0:0.0005:norm(mean_left-mean_left_hip))'*u_lhip;

                
                u_rhip = (mean_right-mean_right_hip)/norm(mean_right-mean_right_hip);
                line_r = mean_right_hip + (0:0.0005:norm(mean_right-mean_right_hip))'*u_rhip;

                
                ptcl_line = pointCloud(line);
                ptcl_line.Color = lab2uint8(repmat([255 255 0],ptcl_line.Count,1));
                
                ptcl_line_l = pointCloud(line_l);
                ptcl_line_l.Color = lab2uint8(repmat([255 255 0],ptcl_line_l.Count,1));
                
                ptcl_line_r = pointCloud(line_r);
                ptcl_line_r.Color = lab2uint8(repmat([255 255 0],ptcl_line_r.Count,1));
                
                ptcl_lrtotal_mean = pointCloud(([x(:),y(:),z(:)]*0.005)+[mean_lrtotal(1) mean_lrtotal(2) mean_lrtotal(3)]);
                ptcl_lrtotal_mean.Color = lab2uint8(repmat([255 255 0],ptcl_lrtotal_mean.Count,1));
                
                angle_r = real(asind((mean_right_hip(2)-mean_right(2))/(mean_right_hip(3)-mean_right(3))));
                angle_l = real(asind((mean_left_hip(2)-mean_left(2))/(mean_left_hip(3)-mean_left(3))));

                view(player,pccat([ptcl_zone ptcl_zone_mean ptcl_line ptcl_lrtotal_mean ...
                                   ptcl_right ptcl_right_mean ptcl_right_hip ptcl_right_hip_mean ptcl_line_r ...
                                   ptcl_left ptcl_left_mean ptcl_left_hip ptcl_left_hip_mean ptcl_line_l]));
                
%                 if angle_r > 30
%                     title.String = 'Right HS-TO';
%                 elseif angle_l > 30
%                     title.String = 'Left HS-TO';
%                 else
%                     title.String = '';
%                 end

                addpoints(li_y,toc,mean_zone(1));
                drawnow limitrate
                subplot(4,1,1)
                subtitle(['Y position = ' num2str(mean_zone(2))])
                xlim([toc-10 toc+10]) 
                ylabel('y [m]')

                angle = real(asind((mean_left(2)-mean_right(2))/(mean_left(1)-mean_right(1))));
                
                addpoints(li_th,toc,angle);
                drawnow limitrate
                subplot(4,1,2)
                subtitle(['Angle position = ' num2str(angle)])
                xlim([toc-10 toc+10])  
                xlabel('t [s]')
                ylabel('\theta [°]')
                
                addpoints(li_th_r,toc,angle_r);
                drawnow limitrate
                subplot(4,1,3)
                subtitle(['Right hip position = ' num2str(angle_r)])
                xlim([toc-10 toc+10])  
                xlabel('t [s]')
                ylabel('\theta [°]')
                
                addpoints(li_th_l,toc,angle_l);
                drawnow limitrate
                subplot(4,1,4)
                subtitle(['Left hip position = ' num2str(angle_l)])
                xlim([toc-10 toc+10])  
                xlabel('t [s]')
                ylabel('\theta [°]')

                range = [mean_zone(2)-0.15 mean_zone(2)+0.15];
                
                
            
            end                      
        end    
    end
end