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
directory=params.plotAllOptDir;
robot=robotI.robot;
base=robotI.base;
ee=robotI.ee;
velLimits=robotI.vel_limits;
period=params.period;
optLevel=params.processTrajOpt;

%read the directories (each directory corresponds to a subject)
dirBrowser=DirectoryBrowser(directory);

%loop through each directory
for i=1:dirBrowser.numFolders
    %find files ending in joints.txt
    fileBrowser=FileBrowser(dirBrowser.folderFullPath(i), '\*c3d.txt');
    for j=1:fileBrowser.numFiles
        %display the current file
        disp(fileBrowser.file(j));
        %read the joints file
        jointsFile=jointsFileName(dirBrowser.folderFullPath(i),...
                fileBrowser.file(j), optLevel);
        [joints]=readJointsFile(jointsFile);
        %read the smooth frames file
        smoothFrames=readFramesFile(smoothFramesFileName(dirBrowser.folderFullPath(i),...
            fileBrowser.file(j)));
        %read toolframe file if it exists
        toolFrameFile=toolframeFileName(dirBrowser.folderFullPath(i),...
                fileBrowser.file(j));
        if isfile(toolFrameFile)
            toolframe=readToolframeFile(toolFrameFile);
        else
            toolframe=robotI.HumerusToolframe;
        end
        %compute any differences between the two
        [errorS,errorX,errorY]=checkErrorUsingJoints(joints,smoothFrames,...
            robot,base,ee,toolframe);
        disp(strcat('Position Error: ', num2str(errorS)));
        disp(strcat('Orientation Error X: ', num2str(errorX)));
        disp(strcat('Orientation Error Y: ', num2str(errorY)));
        %compute the cost
        cost=computeCostFromJoints(joints,velLimits,period);
        disp(strcat('Velocity Cost: ', num2str(cost)));
        %plot the joints
        plotJoints(joints,period,velLimits);
        %place breakpoint here to view each individually
    end
end