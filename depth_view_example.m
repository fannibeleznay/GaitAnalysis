classdef depth_view_example < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                  matlab.ui.Figure
        DepthAX                   matlab.ui.control.UIAxes
        ColorAX                   matlab.ui.control.UIAxes
        PointCloudAX              matlab.ui.control.UIAxes
        StartButton               matlab.ui.control.Button
        StopButton                matlab.ui.control.Button
        Up1                       matlab.ui.control.Button
        Down1                     matlab.ui.control.Button
        Up2                       matlab.ui.control.Button
        Down2                     matlab.ui.control.Button
        Left1                     matlab.ui.control.Button
        Right1                    matlab.ui.control.Button
        Left2                     matlab.ui.control.Button
        Right2                    matlab.ui.control.Button
    end

    properties (Access = private) %, Hidden = true)
        MeTimer            % Timer object
        DrawFlag           % Boolean 
        Cfg                % Realsense.config
        Pipe               % Realsense.pipeline
        Colorizer          % Realsense.colorizer
        PointCloud         % Realsense.pointcloud
        Profile            % Realsense.profile
        Frameset           % Realsense.frameset
        hhDepth            % Image
        hhColor            % Image
        hhCloud
        cy1
        cy2
        cx1
        cx2
        my1
        my2
        mx1
        mx2
        y1line
        y2line
        x1line
        x2line
        
    end

    % Callbacks that handle component events
    methods (Access = private)

        function MeTimerFcn(app,~,~)

        if ( app.DrawFlag == 1 )
           
           % lock drawing process
           app.DrawFlag = 0;
           
           % Get frameset
           app.Frameset = app.Pipe.wait_for_frames();

           % Color Immage
           color_frame = app.Frameset.get_color_frame();
           color_data = color_frame.get_data();
           color_img = permute(reshape(color_data',[3,color_frame.get_width(),color_frame.get_height()]),[3 2 1]);
           [ki,kj] = size(app.hhColor);
           if ki*kj < 1
            app.hhColor = imshow(color_img,'Parent',app.ColorAX,'XData', [1 app.ColorAX.Position(3)], ...
                                 'YData', [1 app.ColorAX.Position(4)]);
            app.y1line = yline(app.cy1,'Color','green','LineWidth',2,'Parent',app.ColorAX);
            app.y2line = yline(app.cy2,'Color','red','LineWidth',2,'Parent',app.ColorAX);
            app.x1line = xline(app.cx1,'Color','cyan','LineWidth',2,'Parent',app.ColorAX);
            app.x2line = xline(app.cx2,'Color','magenta','LineWidth',2,'Parent',app.ColorAX);
           else
            app.hhColor.CData = color_img;
            app.y1line.Value = app.cy1;
            app.y2line.Value = app.cy2;
            app.x1line.Value = app.cx1;
            app.x2line.Value = app.cx2;
           end
           
           % Depth Immage
           depth_frame = app.Frameset.get_depth_frame();
           depth_color = app.Colorizer.colorize(depth_frame);
           depth_data = depth_color.get_data();
           depth_img = permute(reshape(depth_data',[3,depth_color.get_width(),depth_color.get_height()]),[3 2 1]);
           
           [ki,kj] = size(app.hhDepth);
           if ki*kj < 1
            app.hhDepth = imshow(depth_img,'Parent',app.DepthAX);
           else
            app.hhDepth.CData = depth_img;
           end
           
           % PointCloud Graph 
           
           if (depth_frame.logical() && color_frame.logical())
               
               cloud_frame = app.PointCloud.calculate(depth_frame);
               vertices = cloud_frame.get_vertices();
               X =  vertices(:,1,1);
               Y =  vertices(:,3,1);
               Z = -vertices(:,2,1);
               ptcl = pointCloud([X Y Z]);
               ptcl_ds = pcdownsample(ptcl,'random',0.5);
               indices = findPointsInROI(ptcl_ds,...
                    [-0.5 0.5 0 1.5 -0.5 0.4]);
               ptcl_zone = select(ptcl_ds,indices);
               X = ptcl_zone.Location(:,1);
               Y = ptcl_zone.Location(:,2);
               Z = ptcl_zone.Location(:,3);
               mean_dist = num2str(mean(ptcl_zone.Location(:,2)));
               
               plot3(X,Y,Z,'.','Parent',app.PointCloudAX);
               % app.PointCloudAX.Title = mean_dist;
               
               if ki*kj < 1
                    plot3(X,Y,Z,'.','Parent',app.PointCloudAX);
               else
                    app.hhCloud.Colormap = [X Y Z];
                    patch([-0.5 -0.5 0.5 0.5],...
                         [0.1 0.8 0.8 0.1],...
                         app.my2,'red','EdgeColor','red','FaceAlpha',0.1,'Parent',app.PointCloudAX);
                    patch([-0.5 -0.5 0.5 0.5],...
                         [0.1 0.8 0.8 0.1],...
                         app.my1,'green','EdgeColor','green','FaceAlpha',0.1,'Parent',app.PointCloudAX);
                    patch(app.mx1,...
                         [0.1 0.8 0.8 0.1],...
                         [-0.5 -0.5 1.2 1.2],'cyan','EdgeColor','cyan','FaceAlpha',0.1,'Parent',app.PointCloudAX);
                    patch(app.mx2,...
                         [0.1 0.8 0.8 0.1],...
                         [-0.5 -0.5 1.2 1.2],'magenta','EdgeColor','magenta','FaceAlpha',0.1,'Parent',app.PointCloudAX);
               end
               
           end
         
           % unlock drawing process
           app.DrawFlag = 1;
           pause(0.001);   
        end
        
       end

        % Executes after component creation
        function StartUpFunc(app)
               % Create Realsense items

               app.Cfg = realsense.config();
               app.Cfg.enable_stream(realsense.stream.depth,640,360,...
                    realsense.format.z16,30);
               app.Cfg.enable_stream(realsense.stream.color,640,360,...
                    realsense.format.rgb8,30)
               app.Pipe = realsense.pipeline();
               app.Colorizer = realsense.colorizer();
               app.PointCloud = realsense.pointcloud();
               app.Profile = app.Pipe.start(app.Cfg);

               % Create timer object

               kFramePerSecond = 30.0;                                  % Number of frames per second
               Period = double(int64(1000.0 / kFramePerSecond))/1000.0+0.001; % Frame Rate
               
               app.MeTimer = timer(...
                 'ExecutionMode', 'fixedSpacing', ...  % 'fixedRate', ...     % Run timer repeatedly
                 'Period', Period, ...                 % Period (second)
                 'BusyMode', 'drop', ... %'queue',...  % Queue timer callbacks when busy
                 'TimerFcn', @app.MeTimerFcn);         % Specify callback function

               app.DrawFlag = 0;
               app.hhDepth = [];
               app.hhColor = [];
               
        end

        % Button pushed function: start timer
        function onStartButton(app, event)
            % If timer is not running, start it
            if strcmp(app.MeTimer.Running, 'off')
               app.DrawFlag = 1;
               start(app.MeTimer);
            end
        end
        
        function onUp1(app, event)            
            
            app.cy1 = app.cy1 - 5;
            app.my1 = app.my1 + 0.1;
            
                       
        end
        
        function onUp2(app, event)
            
            app.cy2 = app.cy2 - 5;
            app.my2 = app.my2 + 0.1;
                      
        end
        
        function onDown1(app, event)
            
            app.cy1 = app.cy1 + 5;
            app.my1 = app.my1 - 0.1;
            
        end
        
        function onDown2(app, event)
            
            app.cy2 = app.cy2 + 5;
            app.my2 = app.my2 - 0.1;
            
        end
        
        function onRight1(app, event)
            
            app.cx1 = app.cx1 + 5;
            app.mx1 = app.mx1 + 0.1;
            
        end
        
        function onRight2(app, event)
            
            app.cx2 = app.cx2 + 5;
            app.mx2 = app.mx2 + 0.1;
            
        end
        
        function onLeft1(app, event)
            
            app.cx1 = app.cx1 - 5;
            app.mx1 = app.mx1 - 0.1;
            
        end
        
        function onLeft2(app, event)
            
            app.cx2 = app.cx2 - 5;
            app.mx2 = app.mx2 - 0.1;
            
        end

        % Button pushed function: stop timer
        function onStopButton(app, event)
            app.DrawFlag = 0;
            stop(app.MeTimer);
        end

        %Close request UIFigure function
        function UIFigureCloseRequest(app,event)
            app.DrawFlag = 0;
            stop(app.MeTimer);
            delete(app.MeTimer);
            app.Pipe.stop();
            delete(app.Profile);
            delete(app.Colorizer);
            delete(app.Pipe);
            delete(app.Cfg);
            delete(app);
        end

    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('WindowState','maximized','Visible','on');
            app.UIFigure.Name = 'Gait Analysis';
            app.UIFigure.CloseRequestFcn = createCallbackFcn(app,@UIFigureCloseRequest);
            setAutoResize(app,app.UIFigure,false);

            % Create ColorAX
            app.ColorAX = uiaxes(app.UIFigure);
            app.ColorAX.Position = [10 420 640 360];
            
            % Create DepthAX
            app.DepthAX = uiaxes(app.UIFigure);
            app.DepthAX.Position = [10 50 640 360];
            
            % Create PointCloudAX
            app.PointCloudAX = uiaxes(app.UIFigure);
            app.PointCloudAX.Position = [900 125 600 600];
            app.PointCloudAX.XLim = [-0.5 0.5];
            app.PointCloudAX.YLim = [0.1 0.8];
            app.PointCloudAX.ZLim = [-0.5 1.2];

            % Create StartButton
            app.StartButton = uibutton(app.UIFigure, 'push');
            app.StartButton.ButtonPushedFcn = createCallbackFcn(app, @onStartButton, true);
            app.StartButton.IconAlignment = 'center';
            app.StartButton.Position = [10 10 100 20];
            app.StartButton.Text = 'Start';
            
            % Create Up1
            app.Up1 = uibutton(app.UIFigure, 'push');
            app.Up1.ButtonPushedFcn = createCallbackFcn(app, @onUp1, true);
            app.Up1.IconAlignment = 'center';
            app.Up1.Position = [670 750 50 20];
            app.Up1.Text = 'Up';
            app.Up1.FontColor = 'green';
            app.cy1 = 100; 
            app.my1 = [1 1 1 1];
            
            
            % Create Up2
            app.Up2 = uibutton(app.UIFigure, 'push');
            app.Up2.ButtonPushedFcn = createCallbackFcn(app, @onUp2, true);
            app.Up2.IconAlignment = 'center';
            app.Up2.Position = [730 750 50 20];
            app.Up2.Text = 'Up';
            app.Up2.FontColor = 'red';
            app.cy2 = 260;
            app.my2 = [-0.2 -0.2 -0.2 -0.2];
            
            % Create Down1
            app.Down1 = uibutton(app.UIFigure, 'push');
            app.Down1.ButtonPushedFcn = createCallbackFcn(app, @onDown1, true);
            app.Down1.IconAlignment = 'center';
            app.Down1.Position = [670 720 50 20];
            app.Down1.Text = 'Down';
            app.Down1.FontColor = 'green';
            
            % Create Down2
            app.Down2 = uibutton(app.UIFigure, 'push');
            app.Down2.ButtonPushedFcn = createCallbackFcn(app, @onDown2, true);
            app.Down2.IconAlignment = 'center';
            app.Down2.Position = [730 720 50 20];
            app.Down2.Text = 'Down';
            app.Down2.FontColor = 'red';
            
            % Create Right1
            app.Right1 = uibutton(app.UIFigure, 'push');
            app.Right1.ButtonPushedFcn = createCallbackFcn(app, @onRight1, true);
            app.Right1.IconAlignment = 'center';
            app.Right1.Position = [730 670 50 20];
            app.Right1.Text = 'Right';
            app.Right1.FontColor = 'cyan';
            app.cx1 = 120;
            app.mx1 = [-0.3 -0.3 -0.3 -0.3];
            
            % Create Right2
            app.Right2 = uibutton(app.UIFigure, 'push');
            app.Right2.ButtonPushedFcn = createCallbackFcn(app, @onRight2, true);
            app.Right2.IconAlignment = 'center';
            app.Right2.Position = [730 640 50 20];
            app.Right2.Text = 'Right';
            app.Right2.FontColor = 'magenta';
            app.cx2 = 520;
            app.mx2 = [0.3 0.3 0.3 0.3]; 
            
            % Create Left1
            app.Left1 = uibutton(app.UIFigure, 'push');
            app.Left1.ButtonPushedFcn = createCallbackFcn(app, @onLeft1, true);
            app.Left1.IconAlignment = 'center';
            app.Left1.Position = [670 670 50 20];
            app.Left1.Text = 'Left';
            app.Left1.FontColor = 'cyan';
            
            % Create Left2
            app.Left2 = uibutton(app.UIFigure, 'push');
            app.Left2.ButtonPushedFcn = createCallbackFcn(app, @onLeft2, true);
            app.Left2.IconAlignment = 'center';
            app.Left2.Position = [670 640 50 20];
            app.Left2.Text = 'Left';
            app.Left2.FontColor = 'magenta';

            % Create StopButton
            app.StopButton = uibutton(app.UIFigure, 'push');
            app.StopButton.ButtonPushedFcn = createCallbackFcn(app, @onStopButton, true);
            app.StopButton.IconAlignment = 'center';
            app.StopButton.Position = [120 10 100 20];
            app.StopButton.Text = 'Stop';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = depth_view_example

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Set Startup function - after component creation
            runStartupFcn(app,@StartUpFunc);
            
            if nargout == 0
                clear app
            end            
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end

