function waistEstimator_mastro(version)
    clc;
    close all;
    
    ObjDist = 0.45;
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

    % Assuming min dist. = 0.2m and max dist. = 0.7m
    dist1 = 0.20;
    dist2 = 0.70;

    % Recommended cuts for those distances (x).
    min_x1 = -0.20;
    max_x1 = 0.20;
    min_x2 = -0.20;
    max_x2 = 0.20;
    % Recommended cuts for those distances (y).
    min_y1 = 0.10;
    max_y1 = 0.30;
    min_y2 = 0.60;
    max_y2 = 0.80;
    % Recommended cuts for those distances (z).
    min_z1 = -0.20;
    max_z1 = 0.40;
    min_z2 = -0.10;
    max_z2 = 0.15;

    % Linear Fit
    xmin = (ObjDist-dist1)*(min_x2-min_x1)/(dist2-dist1)+min_x1;
    xmax = (ObjDist-dist1)*(max_x2-max_x1)/(dist2-dist1)+max_x1;
    ymin = (ObjDist-dist1)*(min_y2-min_y1)/(dist2-dist1)+min_y1;
    ymax = (ObjDist-dist1)*(max_y2-max_y1)/(dist2-dist1)+max_y1;
    zmin = (ObjDist-dist1)*(min_z2-min_z1)/(dist2-dist1)+min_z1;
    zmax = (ObjDist-dist1)*(max_z2-max_z1)/(dist2-dist1)+max_z1;

                  
    % Main loop
    for iii = 1:10

    % Obtain frames from a streaming device
    fs = pipe.wait_for_frames();

    % Divide in Depth and Color Frame
    depth = fs.get_depth_frame();
    color = fs.get_color_frame();

    % Produce pointcloud
    if (depth.logical() && color.logical())

        pointcloud.map_to(color);
        points = pointcloud.calculate(depth);
        frames = points.get_frame_number();
        switch version
            
            case 1 % Hard calculation of the waist given fixed x,y,z range
                
                % Adjust frame CS to matlab CS 
                vertices = points.get_vertices();
                X =  vertices(640*150:640*300,1,1);
                Y =  vertices(640*150:640*300,3,1);
                Z = -vertices(640*150:640*300,2,1);
            
                count = 1;
                sum = 0;

                for i = 1 : 25 : 640 % hard x range
                    for j = 0 : 25 : 300 % hard y range
                        a = depth.get_distance(i,j);
                        if a < 0.7 && a > 0.2  % hard z range
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
                X =  vertices(:,1,1); %length = 480
                Y =  vertices(:,3,1);
                Z = -vertices(:,2,1);

                
                % Creation of the Point Cloud as Matlab requires
                ptcl = pointCloud([X Y Z]);
                
                % Down Sampling to handle less points
                ptcl_ds = pcdownsample(ptcl,'random',0.5);
                
                % Determine the portion of point cloud to analize
                indices = findPointsInROI(ptcl_ds,...
                    [xmin xmax ymin ymax zmin zmax]);
                
                
                % New pointcloud with defined range     
                ptcl_zone = select(ptcl_ds,indices);
                
                % New X,Y,Z vectors
                X1 = ptcl_zone.Location(:,1);
                Y1 = ptcl_zone.Location(:,2);
                Z1 = ptcl_zone.Location(:,3);
                
                mean_dist = num2str(mean(ptcl_zone.Location(:,2)));
                
                % Video Image              
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

% 
%         Key=get(gcf,'CurrentKey');
%       %using strcmp for string comparison if comparison is true = 1
%         if strcmp(num2str(Key),'')==1
%       %If up arrow is pressed thrust = 1
%         elseif strcmp(num2str(Key),'uparrow')==1
%         xmin=xmin+0.1;
%         end
% 
%         f = figure;
% set(f, 'KeyPressFcn', @(x,y)disp(get(f,'CurrentCharacter')))


        % Display Point Cloud 
        subplot(2,2,2)      
        plot3(X,Y,Z,'.');
        hold on
        grid on
        lines = line([xmin xmin],[ymin ymin],[zmin zmax], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmin xmin],[ymin ymax],[zmin zmin], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmax xmax],[ymin ymin],[zmin zmax], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmax xmax],[ymin ymax],[zmin zmin], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmin xmax],[ymin ymin],[zmin zmin], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmin xmax],[ymin ymin],[zmax zmax], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmin xmin],[ymax ymax],[zmin zmax], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmax xmax],[ymax ymax],[zmin zmax], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmin xmin],[ymax ymax],[zmin zmin], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmin xmax],[ymax ymax],[zmax zmax], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmin xmax],[ymax ymax],[zmin zmin], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmin xmin],[ymin ymax],[zmax zmax], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([xmax xmax],[ymin ymax],[zmax zmax], 'Color', 'Red', 'LineStyle', '--', 'LineWidth', 1);
        view([45 30]);
        hold off

        xlim([-0.5 0.5])
        ylim([0 1])
        zlim([-0.5 0.5])

        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        title(mean_dist)

        % Display Video image 
        subplot(2,2,[1,3])
        imshow(imgout); 
        line([0 1],[0 1]); 
        title(sprintf("Snapshot from %s", name));



        % Display Depth image
        subplot(2,2,4)            
        imshow(imgcol);
        hold on
        lines = line([100 100],[100 200], 'Color', 'White', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([200 200],[100 200], 'Color', 'White', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([100 200],[100 100], 'Color', 'White', 'LineStyle', '--', 'LineWidth', 1);
        lines = line([100 200],[200 200], 'Color', 'White', 'LineStyle', '--', 'LineWidth', 1);
        hold off
        title(sprintf("Colorized depth frame from %s", name));

        pause(0.001);            
    end        
    end

    % Stop streaming
    pipe.stop(); 
              
end

