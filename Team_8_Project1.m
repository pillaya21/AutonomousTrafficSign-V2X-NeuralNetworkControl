clc
clear
close all
fclose(instrfindall);
i=1;
%% communication

IPsystem1 = '198.21.234.24';
% IPsystem2 = '198.21.198.61'; %Shreyas
IPsystem2 = '198.21.204.79'; %Bowen

portSystem1 = 9090;
portSystem2 = 9091;
udpSystem1 = udp(IPsystem2,portSystem2,'LocalPort',portSystem1);
flagSchool = false;
flagStop = false;
fopen(udpSystem1);
%% image for lane detection
Cam=webcam(1);
Cam.Resolution='352x288';
% {'640x480','160x120','176x144','320x240','352x288','800x600','1280x720','1920x1080'}
a=arduino('COM5','UNO','Libraries','servo');
V=servo(a,'D8','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
S=servo(a,'D12','MinPulseDuration',1e-3,'MaxPulseDuration',2e-3);
writePosition(V, 0.5);
writePosition(S, 0.5);
pause(0.05);
Left =[];Right =[];L=[];R=[];Ltheta=zeros(7,1);Rtheta=Ltheta;
vel1=0.564;vel2 = 0.5595;vel=vel2;
iterSchool = 0;dataqueue = zeros(1,5);
Mx=zeros(1,2);
My=Mx;
%% KALMAN FILTER FOR STEERING
ECS=0;EQS=1; ERS=1;
% Errors for Kalman gain for Left and Right Lanes;
EC=[0.001,0.0,0.0,0.0;
    0,0.005,00,00;
    0.0,0.0,0.001,0.0;
    0,00,0,0.0005;];
EP=EC;
EQ=0.035*[0.001,0.0,0.0,0.0;
    0,0.00655,00,00;
    0.0,0.0,0.001,0.0;
    0,00,0,0.00654;];
ER=0.00008*[1.2,0.0,0.0,0.0;
    0,2,00,00;
    0.0,0,1.2,0.0;
    0,00,0,2;];

while (1)
    tic
    if flagStop == true
        pause(2);
        flagStop = false;
    end
    if rem(i,100)==0
        writePosition(V,0.5);
        pause(0.02)
    end
    p=snapshot(Cam);
    p=imresize(p,1);
    shape=size(p);
    p1=rgb2gray(p);
    %detect edges
    for q=1:shape(2)
        for j=1:shape(1)
            if p1(j,q)<150
                p1(j,q)=0;
            end
        end
    end
    
    BW=edge(p1,'canny',[0.3,0.95]);
    %Region Masking
    a1=0.01/6;
    a2=4/6;
    b1=3/7;
    b2=b1;
    a=[shape(2)*a1,shape(2)*a2,shape(2),0];
    b=[shape(1)*b1,shape(1)*b2,shape(1),shape(1)];
    bw=roipoly(p,a,b);
    BW=(BW(:,:)&bw);
    pp=1;qq=1;pt=1;qt=1;
    [H,theta,rho]=hough(BW,'Theta',-60:60); %Transform Display
    
    P = houghpeaks(H,4,'threshold',0.3*max(max((H))));
    %first number of peaks to identify Threshold- minimum value to be
    %considered a peak
    
    lines = houghlines(BW,theta,rho,P,'FillGap',20,'MinLength',5);
    %Minimum -lines shorter than that are omitted
    %When the distance between the line segments is less than the value
    %specified, the houghlines function merges the line segments into
    %a single line segment.
    
    xy=zeros(length(lines),4);
    theta=zeros(length(lines),1);
    for ui=1:length(lines)
        theta(ui)=lines(ui).theta;
    end
    TL=theta>=0;
    Tl=sum(TL);
    Tr=length(lines)-Tl;
    L=zeros(2*Tl,2);R=zeros(2*Tr,2);Ltheta=zeros(Tl);Rtheta=zeros(Tr);
    for k = 1:length(lines)
        xy(k,:) = [lines(k).point1,lines(k).point2];
        theta=lines(k).theta;
        if theta>=0
            L(pp,:)=[lines(k).point1];
            L(pp+1,:)=[lines(k).point2]; %points for left line
            Ltheta(pt)=[lines(k).theta];
            pp=pp+2;pt=pt+1;
        else
            R(qq,:)=[lines(k).point1];
            R(qq+1,:)=[lines(k).point2];   %points for right line
            Rtheta(qt)=[lines(k).theta];
            qq=qq+2;qt=qt+1;
        end
    end
    if ~isempty(L)
        Lx=L(:,1);
        Ly=L(:,2);
        Left=polyfit(L(:,1),L(:,2),1);
    end
    if ~isempty(R)
        Rx=R(:,1);
        Ry=R(:,2);
        Right=polyfit(R(:,1),R(:,2),1);
    end
    Lxx=min(Lx):max(Lx);
    Rxx=min(Rx):max(Rx);
    %     %%     Kalaman Filter for Left and Right
    if i==1
        X=[Left,Right]';
    end
    X0=X;%prediction
    X1=[Left,Right]';%measurement
    EC=EC+EQ;%error covariance
    K=EC/(EC+ER);%kalman gain
    X=X0+K*(X1-X0);%correction
    EC=(eye(4)-K)*EC;%error
    Left=[X(1),X(2)];
    Right=[X(3),X(4)];
    %     %kalman filter for left and right end
    
Steer=SteeringNN(X');
    if Steer>=0.8
        Steer=0.8;
        vel=0.56;
    elseif Steer<=0.2
        Steer=0.2;
        vel=0.56;
    end
    %% Kalman for Steering
    if i==1
        XS=Steer;
    end
    XS0=XS;%prediction
    XS=Steer;    %measurement
    ECS=ECS+EQS;%error covariance
    KS=ECS/(ECS+ERS);%kalman gain
    XS=XS0+KS*(XS-XS0);%correction
    ECS=(eye(1)-KS)*ECS;%error
    if udpSystem1.BytesAvailable> 0
        data = fread(udpSystem1, udpSystem1.BytesAvailable);
        flushinput(udpSystem1)
        if data == 1
            writePosition(V,0.494);
            disp('STOP');
            pause(3);
            vel=vel1;
            flagStop = true;
        elseif data == 2
            disp('School');
            flagSchool = true;
            iterSchool = 0;
            vel=vel2;
        end
    end
    writePosition(V,vel);
    disp(vel)
    writePosition(S,XS);   
    i=i+1;
end


