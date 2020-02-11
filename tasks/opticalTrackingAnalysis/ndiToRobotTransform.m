[path,~,~]=fileparts(mfilename('fullpath'));
addpath(path);
%add initClasses
addpath(fullfile(path,'..','..','initClasses'));
addpath(fullfile(path,'..','..','externalLibs','xml2struct'));

%initialize
params=Parameters(fullfile(path,'..','..','parameters','parametersM20.xml'));
addpath(params.mocapToRobotLibPath);
initMocapToRobotLib();

%parameters
calFolder=params.axisCalibrationFolder;
j1Offset=params.axisCalibrationJ1Offset;
processTcp=params.axisCalibrationTcp;
axisCalResultsFile=params.axisCalibrationResults;

%where are the calibration files
j1File=fullfile(calFolder,'Joint16D.csv');
j2File=fullfile(calFolder,'Joint26D.csv');
j4File=fullfile(calFolder,'Joint46D.csv');
pos6DFile=fullfile(calFolder,'Position6D0.csv');

%perform the actual calibration procedure
[axis,axisC,hemisphereAxes,tcp,residualsX,residualsY,residualsZ,radiusDiff,maxResTcp,rmsResTcp,joint1pos,joint2pos,joint4pos]=...
    computeNDIToRobot(j1File,j2File,j4File,pos6DFile,processTcp,@tcpCalculation);

%compute the euler angles for the hemisphere
hemisphereEul = rad2deg(fliplr(rotm2eul(hemisphereAxes)));

%compute max and rms residual errors
maxRes = zeros(1,3);
rmsRes = zeros(1,3);
maxRes(1) = max(residualsX);
maxRes(2) = max(residualsY);
maxRes(3) = max(residualsZ);
rmsRes(1) = sqrt(sum(residualsX.^2)./length(residualsX));
rmsRes(2) = sqrt(sum(residualsY.^2)./length(residualsY));
rmsRes(3) = sqrt(sum(residualsZ.^2)./length(residualsZ));

%axisC is the rotation matrix from optotrak to the robot, therefore axisC
%transpose is the transformation matrix from optotrak to the robot - which
%is what we want
%we also have account for the potential J1 offset, which would simply be a
%rotation about the z-axis of the robot
undoJ1=axang2rotm([0 0 1 deg2rad(-j1Offset)]);
%needs to be right-multiplied because the rotation is about the 
%coordinate system of the robot
tm=(axisC*undoJ1)';
csvwrite(fullfile(calFolder,'transformation.csv'),tm);

%read calibration files
joint1pos3D = readNDI3D(fullfile(calFolder,'Joint13D.csv'));
joint2pos3D = readNDI3D(fullfile(calFolder,'Joint23D.csv'));
joint4pos3D = readNDI3D(fullfile(calFolder,'Joint43D.csv'));

%determine if data is present
joint1posPresent = squeeze(all(joint1pos3D>-3E28,2));
joint2posPresent = squeeze(all(joint2pos3D>-3E28,2));
joint4posPresent = squeeze(all(joint4pos3D>-3E28,2));

%percentage present
joint1Per=(sum(joint1posPresent,2)/size(joint1pos3D,3))*100;
joint2Per=(sum(joint2posPresent,2)/size(joint2pos3D,3))*100;
joint4Per=(sum(joint4posPresent,2)/size(joint4pos3D,3))*100;

if processTcp
    tcpPos3d = readNDI3D(fullfile(calFolder,'Position3D0.csv'));
    tcpPresent = squeeze(all(tcpPos3d>-3E28,2));
    tcpPer=(sum(tcpPresent,2)/size(tcpPos3d,3))*100;
    max([joint1Per joint2Per joint4Per tcpPer], [], 2)
else
    tcp=[0;0;0];
    radiusDiff=0;
    maxResTcp=0;
    rmsResTcp=0;
    max([joint1Per joint2Per joint4Per], [], 2)
end

csvwrite(fullfile(calFolder,'plotJ1.csv'), (tm*joint1pos')');
csvwrite(fullfile(calFolder,'plotJ2.csv'), (tm*joint2pos')');
csvwrite(fullfile(calFolder,'plotJ4.csv'), (tm*joint4pos')');

if(~isempty(axisCalResultsFile))
    dlmwrite(axisCalResultsFile,[tcp' hemisphereEul maxRes rmsRes maxResTcp rmsResTcp radiusDiff],'delimiter',',','-append');
end

fileID = fopen(fullfile(calFolder,'calSummary.txt'), 'w');
fprintf(fileID, 'Max Residual: %.4f \r\n', maxRes);
fprintf(fileID, 'RMS Residual: %.4f \r\n', rmsRes);
fprintf(fileID, 'Max Residual TCP: %.4f \r\n', maxResTcp);
fprintf(fileID, 'RMS Residual TCP: %.4f \r\n', rmsResTcp);
fprintf(fileID, 'Radius Difference: %.4f \r\n', radiusDiff);
fprintf(fileID, 'J1 Offset: %.4f \r\n', j1Offset);
fclose(fileID);
