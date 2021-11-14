function rappresentation()
    clc;
    close all;
    
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    
    % Make Colorizer object to prettify depth output
    colorizer = realsense.colorizer();
    
    pointcloud = realsense.pointcloud();

    % Start streaming on an arbitrary camera with default settings
    config = realsense.config();
    profile = pipe.start(config);
    
    % Get streaming device's name
    dev = profile.get_device();
    name = dev.get_info(realsense.camera_info.name);
    
    % Initilize Detector
    detector = posenet.PoseEstimator;
    peopleDetector = peopleDetectorACF;
  
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
            [bbox,score] = detect(peopleDetector,img,'SelectStrongest',false);
            [selectedBbox,~] = selectStrongestBbox(bbox,score,'NumStrongest',1);
            [croppedImages, croppedBBoxes] = detector.normalizeBBoxes(img,selectedBbox);
            heatmaps = detector.predict(croppedImages);
            keypoints = detector.heatmaps2Keypoints(heatmaps);
            [imgout,joints] = detector.visualizeKeyPointsMultiple(I,keypoints,croppedBBoxes); 
            dist = zeros(17,3);
            dist(:,1:2) = joints(:,1:2);
            for i = 1 : 17
            dist(i,3) = depth.get_distance(dist(i,1),dist(i,1));
            end
            
            radius = zeros(17,1) + 5;
            imgout = insertObjectAnnotation(imgout,'circle',[dist(:,1:2)' radius'],dist(:,3));
            
            % Display image 
            subplot(2,2,2)
            imshow(imgout); 
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

