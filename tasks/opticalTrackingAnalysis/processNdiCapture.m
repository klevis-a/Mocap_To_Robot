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

%robot info
hstoolframe=robotI.HSToolframe;
bonetoolframe=robotI.HumerusToolframe;
robot=robotI.robot;
base=robotI.base;
ee=robotI.ee;

%processing info
mocapFolder=params.processMocapFolder;
butterworthCutoff=params.butterworthCutoff;
butterworthOrder=params.butterworthOrder;
programName=params.processMocapProgramName;
c3dFile=params.processMocapC3DFile;
mocapSamplingPeriod=params.period;
ndiSamplingPeriod=params.ndiSamplingPeriod;
tickDef=params.tickDefinition;
%posPeakProminence=params.processMocapPeakProminencePos;
%orientPeakProminence=params.processMocapPeakProminenceOrient;
justGraph=params.processMocapJustGraph;
shouldPrint=params.processMocapPrint;

%sampling periods
samplingPeriods.ndiSamplingPeriod=ndiSamplingPeriod;
samplingPeriods.mocapSamplingPeriod=mocapSamplingPeriod;
samplingPeriods.jointsSamplingPeriod=tickDef;

%capture files
captureFiles.postAnalysisResultsFolder=mocapFolder;
captureFiles.ndiFile=ndiFile(mocapFolder,programName);
captureFiles.c3dFile=c3dFile;
captureFiles.transformFile=transformationFile(mocapFolder);
captureFiles.jointsFile=capturedJointsFile(mocapFolder,programName);

%toolframes
toolframes.hstoolframe=hstoolframe;
toolframes.bonetoolframe=bonetoolframe;

kinOpts.zeroRot=1;
captureInfo=struct;
captureInfo.programName=programName;
smoothDataFunctions.ndi=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.ndiSamplingPeriod);
smoothDataFunctions.joints=createButterworthFilter(butterworthOrder,butterworthCutoff,1/samplingPeriods.jointsSamplingPeriod);
captureInfo=readCapture(captureInfo,captureFiles,samplingPeriods,robotI,smoothDataFunctions);
captureInfo=addThoraxCS(captureInfo,captureFiles.c3dFile,samplingPeriods.mocapSamplingPeriod);
captureInfo=calcAllKinematics(captureInfo,captureInfo.sources,captureInfo.referenceCS,captureInfo.rigidBodies,kinOpts);

%For position I will use the hemisphere position, while for orientation I
%will use the bone orientation. This decision stems from the fact that the
%bone position depends on the hemisphere orientation - which can be noisy.
%So transforming from the hemisphere position to the bone position, by
%necessarily using the hemisphere orientation, adds a significant amount of
%noise. On the other hand, transforming from the bone position (mocap
%frames) to the hemisphere position does not add noise because the
%transformation is static

if justGraph
    switch captureInfo.activity
        case Activities.JJ_free
            cs='lab';
            linRB='bone';
            rotRB='bone';
            eulerRB='bone';
        case Activities.JL
            cs='lab';
            linRB='hs';
            rotRB='bone';
            eulerRB='bone';
        case Activities.JO_free
            cs='lab';
            linRB='bone';
            rotRB='bone';
            eulerRB='bone';
        case Activities.IR90
            cs='lab';
            linRB='bone';
            rotRB='bone';
            eulerRB='bone';
    end
    close all
    figure(1)
    plot(captureInfo.mocap.time,captureInfo.mocap.(cs).(linRB).mmdeg.pose.position.scalar);
    title('Mocap Position');
    
    figure(2)
    plot(captureInfo.ndi.time,captureInfo.ndi.(cs).(linRB).mmdeg.pose.position.scalar);
    title('Optotrak Position');
    
    figure(3)
    plot(captureInfo.joints.time,captureInfo.joints.(cs).(linRB).mmdeg.pose.position.scalar);
    title('Joints Position');
    
    figure(4)
    plot(captureInfo.mocap.time,captureInfo.mocap.(cs).(rotRB).mmdeg.pose.rotation.scalar);
    title('Mocap Orientation');
    
    figure(5)
    plot(captureInfo.ndi.time,captureInfo.ndi.(cs).(rotRB).mmdeg.pose.rotation.scalar);
    title('Optotrak Orientation');
    
    figure(6)
    plot(captureInfo.joints.time,captureInfo.joints.(cs).(rotRB).mmdeg.pose.rotation.scalar);
    title('Joints Orientation');
    
%     figure(7)
%     plot(captureInfo.mocap.time,captureInfo.mocap.thorax.(eulerRB).mmdeg.pose.euler.yxz.vector)
%     legend('Angle of Elevation', 'Flexion/Extension', 'Axial Rotation');
%     title('Mocap Euler Angles')
%     
%     figure(8)
%     plot(captureInfo.ndi.time,captureInfo.ndi.thorax.(eulerRB).mmdeg.pose.euler.yxz.vector);
%     legend('Angle of Elevation', 'Flexion/Extension', 'Axial Rotation');
%     title('NDI Euler Angles')
%     
%     figure(9)
%     plot(captureInfo.joints.time,captureInfo.joints.thorax.(eulerRB).mmdeg.pose.euler.yxz.vector);
%     legend('Angle of Elevation', 'Flexion/Extension', 'Axial Rotation');
%     title('Joints Euler Angles')
    return
end

switch captureInfo.activity
    case Activities.JJ_free
        minPosPeak = input('What is the minimum peak height to use for position?');
        [mocapNdiFit, mocapJointsFit]=determineJJTiming(captureInfo,minPosPeak);
    case Activities.JL
        [mocapNdiFit, mocapJointsFit, alignData]=determineJLTiming(captureInfo);
    case Activities.JO_free
        [mocapNdiFit, mocapJointsFit, alignData]=determineJOTiming(captureInfo);
    case Activities.IR90
        [mocapNdiFit, mocapJointsFit, alignData]=determineIRTiming(captureInfo);
end
mocapNdiOffsetIndex = determineOffsetIndex(captureInfo.ndi.time,mocapNdiFit.offset);
mocapJointsOffsetIndex = determineOffsetIndex(captureInfo.joints.time,mocapJointsFit.offset);
%% Plotting
close all
%plot scalar position
figure(1)
plot(captureInfo.ndi.time(mocapNdiOffsetIndex:end)-captureInfo.ndi.time(mocapNdiOffsetIndex),...
    captureInfo.ndi.lab.hs.mmdeg.pose.position.scalar(mocapNdiOffsetIndex:end));
hold on
plot(captureInfo.joints.time(mocapJointsOffsetIndex:end)-captureInfo.joints.time(mocapJointsOffsetIndex),...
    captureInfo.joints.lab.hs.mmdeg.pose.position.scalar(mocapJointsOffsetIndex:end));
hold on
plot(captureInfo.mocap.time,captureInfo.mocap.lab.hs.mmdeg.pose.position.scalar);
legend('Captured Data', 'Joints Data', 'Mocap Data');
title('Scalar position over time');
if shouldPrint
    scalarPosFile=fullfile(mocapFolder,'scalarPos.pdf');
    print(scalarPosFile, '-dpdf')
end

%plot scalar orientation
figure(2)
plot(captureInfo.ndi.time(mocapNdiOffsetIndex:end)-captureInfo.ndi.time(mocapNdiOffsetIndex),...
    captureInfo.ndi.thorax.bone.mmdeg.pose.rotation.scalar(mocapNdiOffsetIndex:end));
hold on
plot(captureInfo.joints.time(mocapJointsOffsetIndex:end)-captureInfo.joints.time(mocapJointsOffsetIndex),...
    captureInfo.joints.thorax.bone.mmdeg.pose.rotation.scalar(mocapJointsOffsetIndex:end));
hold on
plot(captureInfo.mocap.time,captureInfo.mocap.thorax.bone.mmdeg.pose.rotation.scalar);
legend('Captured Data', 'Joints Data','Mocap Data');
title('Scalar orientation over time');
if shouldPrint
    scalarOrientFile=fullfile(mocapFolder,'scalarOrient.pdf');
    print(scalarOrientFile, '-dpdf')
end

%start plotting individual components for position
for n=1:3
    figure(n+2)
    plot(captureInfo.ndi.time(mocapNdiOffsetIndex:end)-captureInfo.ndi.time(mocapNdiOffsetIndex),...
        captureInfo.ndi.lab.hs.mmdeg.pose.position.vector(mocapNdiOffsetIndex:end,n));
    hold on
    plot(captureInfo.joints.time(mocapJointsOffsetIndex:end)-captureInfo.joints.time(mocapJointsOffsetIndex),...
        captureInfo.joints.lab.hs.mmdeg.pose.position.vector(mocapJointsOffsetIndex:end,n));
    hold on
    plot(captureInfo.mocap.time,captureInfo.mocap.lab.hs.mmdeg.pose.position.vector(:,n));
    legend('Captured Data', 'Joints Data', 'Mocap Data');
    switch n
        case 1
            title('X');
            if shouldPrint
                xPosFile=fullfile(mocapFolder,'xPos.pdf');
                print(xPosFile, '-dpdf')
            end
        case 2
            title('Y');
            if shouldPrint
                yPosFile=fullfile(mocapFolder,'yPos.pdf');
                print(yPosFile, '-dpdf')
            end
        case 3
            title('Z');
            if shouldPrint
                zPosFile=fullfile(mocapFolder,'zPos.pdf');
                print(zPosFile, '-dpdf')
            end
    end
end

%start plotting individual components for rotation
for n=1:3
    figure(n+5)
    plot(captureInfo.ndi.time(mocapNdiOffsetIndex:end)-captureInfo.ndi.time(mocapNdiOffsetIndex),...
        captureInfo.ndi.thorax.bone.mmdeg.pose.rotation.vector(mocapNdiOffsetIndex:end,n));
    hold on
    plot(captureInfo.joints.time(mocapJointsOffsetIndex:end)-captureInfo.joints.time(mocapJointsOffsetIndex),...
        captureInfo.joints.thorax.bone.mmdeg.pose.rotation.vector(mocapJointsOffsetIndex:end,n));
    hold on
    plot(captureInfo.mocap.time,captureInfo.mocap.thorax.bone.mmdeg.pose.rotation.vector(:,n));
    legend('Captured Data', 'Joints Data', 'Mocap Data');
    switch n
        case 1
            title('X Rot');
            if shouldPrint
                xOrientFile=fullfile(mocapFolder,'xOrient.pdf');
                print(xOrientFile, '-dpdf')
            end
        case 2
            title('Y Rot');
            if shouldPrint
                yOrientFile=fullfile(mocapFolder,'yOrient.pdf');
                print(yOrientFile, '-dpdf')
            end
        case 3
            title('Z Rot');
            if shouldPrint
                zOrientFile=fullfile(mocapFolder,'zOrient.pdf');
                print(zOrientFile, '-dpdf')
            end
    end
end

%plot euler angles
for n=1:3
    figure(8+n)
    plot(captureInfo.ndi.time(mocapNdiOffsetIndex:end)-captureInfo.ndi.time(mocapNdiOffsetIndex),...
        captureInfo.ndi.thorax.bone.mmdeg.pose.euler.yxz.vector(mocapNdiOffsetIndex:end,n));
    hold on
    plot(captureInfo.joints.time(mocapJointsOffsetIndex:end)-captureInfo.joints.time(mocapJointsOffsetIndex),...
        captureInfo.joints.thorax.bone.mmdeg.pose.euler.yxz.vector(mocapJointsOffsetIndex:end,n));
    hold on
    plot(captureInfo.mocap.time,captureInfo.mocap.thorax.bone.mmdeg.pose.euler.yxz.vector(:,n));
    legend('Captured Data', 'Joints Data', 'Mocap Data');
    switch n
        case 1
            title('Angle of Elevation');
        case 2
            title('Plane of Elevation');
        case 3
            title('Axial Rotation');
    end
end

%let's make sure that the starting position of ndi and mocap agree
%this is a little bit more complicated because we allow variation of
%rotation about the z-axis
mocapFirstFrame=squeeze(captureInfo.mocap.lab.bone.pose.frames(1:3,1:3,1));
ndiFirstFrame=squeeze(captureInfo.ndi.robot.bone.pose.frames(1:3,1:3,1));
frameDiff=frameDiffZFree(ndiFirstFrame,mocapFirstFrame);
%using the calculated z-rotation perform rotation of the mocap frame
mocapRotFrame=frameDiff'*mocapFirstFrame;
%now determine the rotation between the two frames
rotDiff=mocapRotFrame*ndiFirstFrame';
rotDiffVec=rotm2axang(rotDiff);
rotAngle = rotDiffVec(4);    
ndiMocapRotAngle=rad2deg(rotAngle);

jointNdiStartOrientDiff=squeeze(captureInfo.joints.robot.hs.pose.frames(1:3,1:3,1))*squeeze(captureInfo.ndi.robot.hs.pose.frames(1:3,1:3,1))';
jointNdiStartOrientVecDiff = rotm2axang(jointNdiStartOrientDiff);
jointNdiRotAngle = rad2deg(jointNdiStartOrientVecDiff(4));

if shouldPrint
    %print summary info to file
    fileID = fopen(fullfile(mocapFolder,strcat(programName,'_sum.txt')), 'w');
    fprintf(fileID, 'Joint NDI Starting Pose angle difference: %.4f \r\n', jointNdiRotAngle);
    fprintf(fileID, 'NDI Mocap Starting Pose angle difference: %.4f \r\n', ndiMocapRotAngle);
    fprintf(fileID, 'Multiplier is: %.4f \r\n', mocapNdiFit.slope);
    fclose(fileID);
end

%print summary info to console
disp(strcat('Joint NDI Starting Pose angle difference: ', num2str(jointNdiRotAngle)));
disp(strcat('NDI Mocap Starting Pose angle difference: ', num2str(ndiMocapRotAngle)));
disp(strcat('Multiplier is: ', num2str(mocapNdiFit.slope)));
disp(strcat('R^2 is: ', num2str(mocapNdiFit.Rsq)));

if shouldPrint
    summaryFileP=fullfile(mocapFolder,strcat(programName,'.csv'));
    fileId = fopen(summaryFileP, 'w');
    splitout=regexp(c3dFile,filesep,'split');
    c3dFilePrint=fullfile(splitout{end-1},splitout{end});
    switch captureInfo.activity
        case Activities.JJ_free
            fprintf(fileId,'c3dFile,posMinPeak\r\n');
            fprintf(fileId,'%s,%.4f\r\n',c3dFilePrint,minPosPeak);
        case Activities.JL
            fprintf(fileId,'c3dFile,ndiMocapPathI,jointsMocapPathI\r\n');
            fprintf(fileId,'%s,%s,%s\r\n',c3dFilePrint,num2str(alignData.ndiMocapPathI),num2str(alignData.jointsMocapPathI));
        case Activities.JO_free
            fprintf(fileId,'c3dFile,ndiMocapPathI,jointsMocapPathI\r\n');
            fprintf(fileId,'%s,%s,%s\r\n',c3dFilePrint,num2str(alignData.ndiMocapPathI),num2str(alignData.jointsMocapPathI));
        case Activities.IR90
            fprintf(fileId,'c3dFile,ndiMocapPathI,jointsMocapPathI\r\n');
            fprintf(fileId,'%s,%s,%s\r\n',c3dFilePrint,num2str(alignData.ndiMocapPathI),num2str(alignData.jointsMocapPathI));
    end
    fclose(fileId);
    
    summaryFile=fullfile(mocapFolder,strcat(programName,'.pdf'));
    if exist(summaryFile, 'file')==2
        delete(summaryFile);
    end
    append_pdfs(summaryFile,scalarPosFile,scalarOrientFile,xPosFile,yPosFile,...
        zPosFile,xOrientFile,yOrientFile,zOrientFile);
    
    delete(scalarPosFile)
    delete(scalarOrientFile)
    delete(xPosFile)
    delete(yPosFile)
    delete(zPosFile)
    delete(xOrientFile)
    delete(yOrientFile)
    delete(zOrientFile)
end