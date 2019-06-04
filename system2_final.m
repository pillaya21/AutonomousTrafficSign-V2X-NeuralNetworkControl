clear;
close all
fclose(instrfindall);

IPsystem1 = '198.21.234.24';
IPsystem2 = '198.21.198.61';
portSystem1 = 9090;
portSystem2 = 9091;
udpSystem2 = udp(IPsystem1,portSystem1,'LocalPort',portSystem2);
fopen(udpSystem2);

cam = webcam(1);
cam.Resolution= '320x240';

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

while (1)
    videoFrame=snapshot(cam);
%     sh=size(videoFrame);
    gray = rgb2gray(videoFrame);
    
    bboxStop = step(detectorStop,gray);
    
    bboxSchool = step(detectorSchool,gray);

%% Corner detection within bounding box
    numStops = 0;
%     numSchools = 0;
    for i = 1:size(bboxStop,1)
        points = detectMinEigenFeatures(gray, 'ROI', bboxStop(i, :));
        xyPointsS1= points.Location;
        if size(xyPointsS1,1) >40
            numStops = numStops +1;
        end
    end
%     for i = 1:size(bboxSchool,1)
%         points = detectMinEigenFeatures(gray, 'ROI', bboxSchool(i, :));
%         xyPoints2= points.Location;        
%             numSchools = numSchools +1;
%     end
    numSchools = size(bboxSchool,1);
    
%%
    if(numStops == 0 && numSchools == 0)
        flagArr = [flagArr(2:end),0];
    elseif(numStops>numSchools)
        flagArr = [flagArr(2:end),1];
    else
        flagArr = [flagArr(2:end),2];
    end
    if(sum(flagArr == 1)>2 )
        %Stop
        fwrite(udpSystem2,1);
        disp('stop')
        pause(5)
        flagArr = zeros(1,5);
    elseif(sum(flagArr == 2)>=1)
        %School
        fwrite(udpSystem2,2);
        disp('school')
        pause(2)
        flagArr = zeros(1,5);
    end
end
