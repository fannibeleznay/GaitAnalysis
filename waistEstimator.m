function waistEstimator(version)
    clc;
    close all;
    
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    
    % Make Colorizer object to prettify depth output
    colorizer = realsense.colorizer();
   
    % Create an empty Point Cloud from the RealSense
    pointcloud = realsense.pointcloud();

    % Start streaming on an arbitrary camera with default settings
    config = realsense.config();
    config.enable_stream(realsense.stream.depth,640,360,...
        realsense.format.z16,30);
    config.enable_stream(realsense.stream.color,640,360,...
        realsense.format.rgb8,30)
    profile = pipe.start(config);
    
    % Get streaming device's name
    dev = profile.get_device();
    name = dev.get_info(realsense.camera_info.name);
  
    % Initialize the Rappresentation
    figure('visible','on','units','normalized','outerposition',[0 0 1 1]);
    
    switch version
        
        case 3
            
            % Initilize Detector
            detector = posenet.PoseEstimator;
            peopleDetector = peopleDetectorACF;
        
    end
                  
    % Main loop
    for iii = 1:10

    % Obtain frames from a streaming device
    fs = pipe.wait_for_frames();

    % Divide in Depth and Color Frame
    depth = fs.get_depth_frame();
    color = fs.get_color_frame();

    % Produce pointcloud
    if (depth.logical() && color.logical())

        pointcloud.map_to(color); % check what this line does
        points = pointcloud.calculate(depth);

        switch version
            
            case 1 % Hard calculation of the waist given fixed x,y,z range
                
                % Adjust frame CS to matlab CS 
                vertices = points.get_vertices();
                X =  vertices(640*150:640*300,1,1);
                Y =  vertices(640*150:640*300,3,1);
                Z = -vertices(640*150:640*300,2,1);
            
                count = 1;
                sum = 0;

                for i = 1 : 25 : 639 % hard x range
                    for j = 150 : 25 : 300 % hard y range
                        a = depth.get_distance(i,j);
                        if a < 1.2 % hard z range
                            sum = sum + a;
                            count = count + 1;
                        end
                    end
                end

                mean_dist = num2str(sum/count);
                
                % Video Immage              
                data = color.get_data();
                imgout = permute(reshape(data',[3,color.get_width(),...
                    color.get_height()]),[3 2 1]);
                
                % Colorize depth frame
                depthcol = colorizer.colorize(depth);

                % Get actual data and convert into a format imshow can use
                % (Color data arrives as [R, G, B, R, G, B, ...] vector)
                datacol = depthcol.get_data();
                imgcol = permute(reshape(datacol',[3,...
                    depthcol.get_width(),depthcol.get_height()]),[3 2 1]);
                
            case 2 % x z Ranges calculated on the Point Cloud
                
                % Adjust frame CS to matlab CS (.Location function)
                vertices = points.get_vertices();
                X =  vertices(:,1,1);
                Y =  vertices(:,3,1);
                Z = -vertices(:,2,1);
                
                % Creation of the Point Cloud as Matlab requires
                ptcl = pointCloud([X Y Z]);
                
                % Down Sampling to handle less points
                ptcl_ds = pcdownsample(ptcl,'random',0.5);
                
                % Determine the portion of point cloud to analize
                indices = findPointsInROI(ptcl_ds,...
                    [-0.5 0.5 0 1.5 -0.5 0.4]);
                
                
                % New pointcloud with defined range     
                ptcl_zone = select(ptcl_ds,indices);
                
                % New X,Y,Z vectors
                X = ptcl_zone.Location(:,1);
                Y = ptcl_zone.Location(:,2);
                Z = ptcl_zone.Location(:,3);
                
                mean_dist = num2str(mean(ptcl_zone.Location(:,2)));
                
                % Video Immage              
                data = color.get_data();
                imgout = permute(reshape(data',[3,color.get_width(),...
                    color.get_height()]),[3 2 1]);
                
                % Colorize depth frame
                depthcol = colorizer.colorize(depth);

                % Get actual data and convert into a format imshow can use
                % (Color data arrives as [R, G, B, R, G, B, ...] vector)
                datacol = depthcol.get_data();
                imgcol = permute(reshape(datacol',[3,...
                    depthcol.get_width(),depthcol.get_height()]),[3 2 1]);
                
            case 3 % Evaluation based on PoseEstimation
                
                % Adjust frame CS to matlab CS
                vertices = points.get_vertices();
                X = -vertices(:,2,1);
                Y =  vertices(:,3,1);
                Z =  vertices(:,1,1);
                
                mean_dist = 'Point Cloud';
                
                % Video Immage     
                data = color.get_data();
                img = permute(reshape(data',[3,color.get_width(),...
                    color.get_height()]),[3 2 1]);
                img = imrotate(img,90);
                [bbox,score] = detect(peopleDetector,img,...
                    'SelectStrongest',false);
                [selectedBbox,~] = selectStrongestBbox(bbox,score,...
                    'NumStrongest',1);
                [croppedImages, croppedBBoxes] = ...
                    detector.normalizeBBoxes(img,selectedBbox);
                % cropped immages must be 256x192x3 uint8
                
                if ~isempty(croppedImages)
                    
                    heatmaps = detector.predict(croppedImages);
                    keypoints = detector.heatmaps2Keypoints(heatmaps);
                    [imgout,joint] = detector.visualizeKeyPointsMultiple...
                        (img,keypoints,croppedBBoxes); 
                    dist = zeros(4,3);
                    dist(:,1:2) = round(joint(12:15,1:2));
                
                    if (prod(dist(:,2) > 0) && prod(dist(:,1) > 0))
                
                        for i = 1 : 4
                            dist(i,3) = depth.get_distance(dist(i,1),...
                                dist(i,1));
                        end

                        radius = zeros(4,1) + 5;
                        imgout = insertObjectAnnotation(imgout,'circle',...
                            [dist(:,1:2) radius],dist(:,3),...
                                'TextBoxOpacity',0.9,'FontSize',30);
                    end
                else
                    imgout = img;
                end
                
                % Colorize depth frame
                depthcol = colorizer.colorize(depth);
                
                % Get actual data and convert into a format imshow can use
                % (Color data arrives as [R, G, B, R, G, B, ...] vector)
                datacol = depthcol.get_data();
                imgcol = permute(reshape(datacol',[3,...
                    depthcol.get_width(),depthcol.get_height()]),[3 2 1]);
                imgcol = imrotate(imgcol,90);
        end

        % Display Point Cloud 
        subplot(2,2,2)      
        plot3(X,Y,Z,'.');
        grid on
        view([45 30]);

        xlim([-0.5 0.5])
        ylim([0.3 2])
        zlim([-0.5 1.2])

        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        title(mean_dist)

        % Display Video image 
        subplot(2,2,[1,3])
        imshow(imgout); 
        title(sprintf("Snapshot from %s", name));

        % Display Depth image
        subplot(2,2,4)            
        imshow(imgcol);
        title(sprintf("Colorized depth frame from %s", name));

        pause(0.001);            
    end        
    end

    % Stop streaming
    pipe.stop(); 
              
end

