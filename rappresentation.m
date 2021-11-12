function rappresentation()
    clc;
    close all;
    
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    
    % Make Colorizer object to prettify depth output
    colorizer = realsense.colorizer();
    
    pointcloud = realsense.pointcloud();

    % Start streaming on an arbitrary camera with default settings
    profile = pipe.start();
    
    % Get streaming device's name
    dev = profile.get_device();
    name = dev.get_info(realsense.camera_info.name);
    
    % Initilize Detector
    detector = posenet.simplePoseEstimator;
  
    figure('visible','on','units','normalized','outerposition',[0 0 1 1]);
    
    % Main loop
    for i = 1:10000

        % Obtain frames from a streaming device
        fs = pipe.wait_for_frames();
        
        % Select depth frame
        depth = fs.get_depth_frame();
        color = fs.get_color_frame();

        % Produce pointcloud
        if (depth.logical() && color.logical())

            pointcloud.map_to(color);
            points = pointcloud.calculate(depth);
            
            % Adjust frame CS to matlab CS
            vertices = points.get_vertices();
            X = vertices(:,1,1);
            Y = vertices(:,2,1);
            Z = vertices(:,3,1);

            subplot(2,2,[1,3])      
            plot3(X,Z,-Y,'.');
            grid on
            view([45 30]);
            % view([0 30]);

            xlim([-0.5 0.5])
            ylim([0.2 1])
            zlim([-0.5 0.5])

            xlabel('X');
            ylabel('Z');
            zlabel('Y');
            
            % Video Immage     
            
            data = color.get_data();
            img = permute(reshape(data',[3,color.get_width(),color.get_height()]),[3 2 1]);
            img = imresize(img, [256 192]);
            
            keypoints = detectPose(detector,img);
            J = detector.visualizeKeyPoints(img,keypoints);     
            J = imresize(J,[480 640]);
            
            % Display image 
            subplot(2,2,2)
            imshow(J); 
            title(sprintf("Snapshot from %s", name));
                        
            % Colorize depth frame
            depthcol = colorizer.colorize(depth);

            % Get actual data and convert into a format imshow can use
            % (Color data arrives as [R, G, B, R, G, B, ...] vector)
            datacol = depthcol.get_data();
            imgcol = permute(reshape(datacol',[3,depthcol.get_width(),depthcol.get_height()]),[3 2 1]);

            % Display image
            subplot(2,2,4)            
            imshow(imgcol);
            title(sprintf("Colorized depth frame from %s", name));
            
            pause(0.001);            
        end
         
    end

    % Stop streaming
    pipe.stop();
    
end

