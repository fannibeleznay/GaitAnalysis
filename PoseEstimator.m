function PoseEstimator()
    clc
    close all
    detector = posenet.PoseEstimator;
    I = imread('WIN_20211114_13_47_36_Pro.jpg');
    imshow(I);
    peopleDetector = peopleDetectorACF;
    
    [bbox,score] = detect(peopleDetector,I,'SelectStrongest',false);
    [selectedBbox,selectedScore] = selectStrongestBbox(bbox,score,'NumStrongest',1);
    I1 = insertObjectAnnotation(I,'rectangle',bbox,score,'Color','r');
    I2 = insertObjectAnnotation(I,'rectangle',selectedBbox,selectedScore,'Color','r');

    figure, imshow(I1);
    title('Detected people and detection scores before suppression'); 
    figure, imshow(I2);
    title('Detected people and detection scores after suppression');
    
    [croppedImages, croppedBBoxes] = detector.normalizeBBoxes(I,selectedBbox);
    Iout2 = insertObjectAnnotation(I,'rectangle',croppedBBoxes,selectedScore,'LineWidth',3);
    imshow(Iout2);
    title('Resize the bounding boxes to be the same aspect ratio in the input of the network.');
    figure, montage(croppedImages);
    title('Each cropped image')
    heatmaps = detector.predict(croppedImages);
    Iheatmaps = detector.visualizeHeatmaps(heatmaps, croppedImages);
    montage(Iheatmaps);
    title("Joint heatmaps")
    keypoints = detector.heatmaps2Keypoints(heatmaps);
    Iheatmaps = detector.visualizeKeyPoints(Iheatmaps,keypoints);
    montage(Iheatmaps);
    title('Extracted joints for each person');
    [Iout3,joints] = detector.visualizeKeyPointsMultiple(I,keypoints,croppedBBoxes);
    th = 0:pi/50:2*pi;
    r = 5;
    for i = 1 : length(joints(:,1))
    xunit(i,:) = r * cos(th) + joints(i,1);
    yunit(i,:) = r * sin(th) + joints(i,2);
    end
    
    imshow(Iout3);
    hold on
    plot(xunit, yunit);
    hold off
    title('Estimated keypoints in the coordinates of the original image');
    
end

