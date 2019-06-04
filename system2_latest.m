clear;
close all
fclose(instrfindall);

IPsystem1 = '198.21.234.24';
IPsystem2 = '198.21.198.61';
portSystem1 = 9090;
portSystem2 = 9091;
udpSystem2 = udp(IPsystem1,portSystem1,'LocalPort',portSystem2);
fopen(udpSystem2);

cam = webcam(2);
cam.Resolution= '320x240';
% videoFrame= snapshot(cam);
% frameSize= size(videoFrame);
% videoPlayer= vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

falseStop = 0.1;
cascadeStop = 10;
falseSchool = 0.1;
cascadeSchool = 10;
xmlStop = char(strcat("stop1","_",num2str(falseStop),"_",num2str(cascadeStop),".xml"));
xmlSchool = char(strcat("school1","_",num2str(falseSchool),"_",num2str(cascadeSchool),".xml"));
detectorStop = vision.CascadeObjectDetector(xmlStop);
detectorSchool = vision.CascadeObjectDetector(xmlSchool);

flagStop = false;
flagSchool = false;
flagNothing = false;
flagArr = zeros(1,5);
ii = 1;
while (1)
%     ii = ii + 1
    videoFrame=snapshot(cam);
    sh=size(videoFrame);
    %     videoFrame=imresize(videoFrame,0.25);
    gray = rgb2gray(videoFrame);
    
        bboxStop = step(detectorStop,gray);
        bbStop = size(bboxStop,1);
%         videoFrameStop = insertObjectAnnotation(videoFrame,'rectangle',bboxStop,'stop sign');
%         step(videoPlayer, videoFrameStop);
    
    bboxSchool = step(detectorSchool,gray);
    bbSchool = size(bboxSchool,1);
%     
%     videoFrameSchool = insertObjectAnnotation(videoFrame,'rectangle',bboxStop,'stop sign');  
%     videoFrameSchool = insertObjectAnnotation(videoFrameSchool,'rectangle',bboxSchool,'school sign');
%     step(videoPlayer, videoFrameSchool);
% figure(01);
% imshow(videoFrameSchool);
% hold on;
%             videoFrame = insertObjectAnnotation(videoFrame,'rectangle',bboxStop,'stop sign','Color','cyan');  
%             videoFrame = insertObjectAnnotation(videoFrame,'rectangle',bboxSchool,'school sign','Color','yellow');

%% Corner detection within bounding box
    numStops = 0;
    numSchools = 0;
%     bboxStop1 = zeros(1,4);
%     bboxStop1(1,:) = [];
%     bboxSchool1 = bboxStop1;
    for i = 1:size(bboxStop,1)
        points = detectMinEigenFeatures(gray, 'ROI', bboxStop(i, :));
        xyPointsS1= points.Location;
%                     videoFrame = insertMarker(videoFrame,xyPointsS1);
        if size(xyPointsS1,1) >40
            numStops = numStops +1;
%             bboxStop1 = [bboxStop1;bboxStop(i, :)];
        end
    end
    for i = 1:size(bboxSchool,1)
        points = detectMinEigenFeatures(gray, 'ROI', bboxSchool(i, :));
        xyPoints2= points.Location;
%                     videoFrame = insertMarker(videoFrame,xyPoints2);
%         if size(xyPoints2,1) >30
            numSchools = numSchools +1;
%             bboxSchool1 = [bboxSchool1;bboxSchool(i, :)];
%         end
    end
%                     step(videoPlayer, videoFrame);
 
%%
    if(numStops == 0 && numSchools == 0)
%         flagNothing = true;
        flagArr = [flagArr(2:end),0];
%         temp(ii) = 0;
    elseif(numStops>numSchools)
%         flagStop = true;
        flagArr = [flagArr(2:end),1];
%         temp(ii) = 1;
    else
%         flagSchool = true;
        flagArr = [flagArr(2:end),2];
%         temp(ii) = 2;
    end
%     && sum(flagArr(end-1:end) == 0) == 2
    if(sum(flagArr == 1)>2 )
        %Stop
        fwrite(udpSystem2,1);
        disp('stop')
        pause(5)
        flagArr = zeros(1,5);
%         size(flagArr,1)/2
    elseif(sum(flagArr == 2)>=1)
        %School
        fwrite(udpSystem2,2);
        disp('school')
        pause(2)
        flagArr = zeros(1,5);
    end
    %     if(bbStop == 0 && bbSchool == 0)
    %         falgNothing = true;
    %     elseif(bbStop == 1 && bbSchool == 0)
    %         flagStop = true;
    %     elseif(bbStop == 0 && bbSchool == 1)
    %         flagSchool = true;
    %     else
    %         % There are multiple bounding boxes in the scene. Reduce them.
    %     end
    
    %     if flagStop == true
    %         fwrite(udpB,1);
    %     elseif flagSchool == true
    %         fwrite(udpSystem2,2);
    %     else
    %         fwrite(udpSystem2,0);
    %     end
    ii = ii + 1;
    drawnow
end
