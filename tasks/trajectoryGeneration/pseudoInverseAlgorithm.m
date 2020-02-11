[path,~,~]=fileparts(mfilename('fullpath'));
addpath(path);
%add initClasses
addpath(fullfile(path,'..','..','initClasses'));
addpath(fullfile(path,'..','..','externalLibs','xml2struct'));

%initialize
params=Parameters(fullfile(path,'..','..','parameters','parametersM20.xml'));
addpath(params.mocapToRobotLibPath);
initMocapToRobotLib();
robotI=RobotInfo(fullfile(path,'..','..','parameters','robotM20.xml'));

%parameters
robot=robotI.robot;
base=robotI.base;
ee=robotI.ee;
desFramesFile=params.pInvDesFramesFile;
joints0=params.pInvJoints0;

%read the desired frames
desFrames=readFramesFile(desFramesFile);

%compute both the intrinsic and extrinsic difference
[desFramesDxI,desFramesDxE] = cumFrameDiff(desFrames);

%generate trajectory using intrinsic frame differences
[desXI,xAchievedI,thetaI] = trajGenI(joints0,desFramesDxI,robot,base,ee);

%generate trajectory using extrinsic frame differences
[desXE,xAchievedE,thetaE] = trajGenE(joints0,squeeze(desFrames(:,:,1)),desFramesDxE,robot,base,ee);

%compare the intrinsic versus extrinsic frames
[diffVector,posError,orientError]=FSError(xAchievedI,xAchievedE);
totalPosError = sum(posError);
totalOrientError = sum(orientError);

%compare the desired frames against intrinsic method
iFramesDiff=cumFrameDiff(xAchievedI);
iVectorDiff=frameToVectorDiff(iFramesDiff);
desVectorDiff=frameToVectorDiff(desFramesDxI);
vecDiff=desVectorDiff-iVectorDiff;
posError=dot(vecDiff(:,1:3),vecDiff(:,1:3),2);
orientError=dot(vecDiff(:,4:6),vecDiff(:,4:6),2);
posErrorT=sum(posError);
orientErrorT=sum(orientError);

function desX = generateDesiredI(start,T)
    %generate a desired trajectory given a sequence of frame differences
    %and a starting position
    dxL=size(T,3);
    desX = zeros(4,4,dxL+1);
    desX(:,:,1)=start;
    for i=1:dxL
        desX(:,:,i+1) = start*T(:,:,i);
    end
end

function desX = generateDesiredE(start,firstFrame,T)
    %generate a desired trajectory given a sequence of frame differences
    %(extrinsic), a start position, and the first frame in the
    %initial frame sequence from which the differences were generated
    dxL=size(T,3);
    desX = zeros(4,4,dxL+1);
    desX(:,:,1)=start;
    initialToCurrent = start*htInverse(firstFrame);
    currentToInitial = htInverse(initialToCurrent);
    for i=1:dxL
        desX(:,:,i+1) = initialToCurrent*T(:,:,i)*currentToInitial*start;
    end
end

function [desX,xAchieved,theta] = trajGenI(theta0,T,robot,base,ee)
    %generate a joint space trajectory using the linear combination of
    %vectors transform
    cartPos0 = getTransform(robot,createConfig(robot,theta0),ee,base);
    desX=generateDesiredI(cartPos0,T);
    [xAchieved,theta]=generateTrajectory(theta0,desX,robot,base,ee);
end

function [desX,xAchieved,theta] = trajGenE(theta0,firstFrame,T,robot,base,ee)
    %generate a joint space trajectory using the active transforms
    cartPos0 = getTransform(robot,createConfig(robot,theta0),ee,base);
    desX=generateDesiredE(cartPos0,firstFrame,T);
    [xAchieved,theta]=generateTrajectory(theta0,desX,robot,base,ee);
end

function [xAchieved,theta] = generateTrajectory(theta0,desX,robot,base,ee)
    %generated a trajectory using a start orientation, a desired dx
    %(intrinsic dx)
    trajL = size(desX,3);
    xAchieved=zeros(4,4,trajL);
    theta=zeros(trajL,6);
    xAchieved(:,:,1)=desX(:,:,1);
    theta(1,:)=theta0;
    for i=1:trajL-1
        jacob = geometricJacobian(robot,createConfig(robot,theta(i,:)),ee);
        csDx = computeDiffVector(xAchieved(:,:,i), desX(:,:,i+1));
        dtheta=pinv(jacob)*csDx;
        theta(i+1,:)=theta(i,:)+dtheta';
        xAchieved(:,:,i+1)=getTransform(robot,createConfig(robot,theta(i+1,:)),ee,base);
    end
end

function diffV=computeDiffVector(frame1,frame2)
    %computes the difference between two frames in vector form
    diffV = zeros(6,1);
    diffV(4:6)=frame2(1:3,4)-frame1(1:3,4);
    matDiff = frame2(1:3,1:3)*frame1(1:3,1:3)';
    axangDiff = rotm2axang(matDiff);
    diffV(1:3) = axangDiff(1:3).*axangDiff(4);
end

function [diffVector, posError, orientError]=FSError(frameSeq1,frameSeq2)
    %computes the position and orientation error in a frame sequence
    diffVector = zeros(length(frameSeq1),6);
    for i=1:length(frameSeq1)
        diffVector(i,:)=computeDiffVector(squeeze(frameSeq1(:,:,i)),squeeze(frameSeq2(:,:,i)));
    end
    posError=dot(diffVector(:,1:3),diffVector(:,1:3),2);
    orientError=dot(diffVector(:,4:6),diffVector(:,4:6),2);
end