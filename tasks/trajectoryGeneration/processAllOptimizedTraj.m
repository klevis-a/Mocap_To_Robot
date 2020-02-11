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
directory=params.processAllOptDir;
period=params.period;
robot=robotI.robot;
base=robotI.base;
ee=robotI.ee;
velLimits=robotI.vel_limits;
optLevel=params.processTrajOpt;

%read the directories (each directory corresponds to a subject)
dirBrowser=DirectoryBrowser(directory);

%store initial joints
initialJoints=zeros(1,6);
%store cost and errors associated with trajectory
costErrors=zeros(1,5);
%stores names associated with cost and initial joints
costNames={};
%store a summary of violations, column one has the number of times over 99%
%of joint speed, column 2 has the max speed, column 3 has the # of frames
violationSummary=[];
%stores names of files associated with violations
violNames={};

counter=1;
violCounter=1;
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
        if ~isfile(jointsFile)
            continue;
        end
        [joints]=readJointsFile(jointsFile);
        %read toolframe file if it exists
        toolFrameFile=toolframeFileName(dirBrowser.folderFullPath(i),...
                fileBrowser.file(j));
        if isfile(toolFrameFile)
            toolframe=readToolframeFile(toolFrameFile);
        else
            toolframe=robotI.HumerusToolframe;
        end
        %read the smooth frames file
        fFile=smoothFramesFileName(dirBrowser.folderFullPath(i),fileBrowser.file(j));
        smoothFrames=readFramesFile(fFile);
        %compute any differences between the two
        [errorS,errorX,errorY]=checkErrorUsingJoints(joints,smoothFrames,...
            robot,base,ee,toolframe);
        %compute the cost
        [cost,velper]=computeCostFromJoints(joints,velLimits,period);
        
        %store results
        initialJoints(counter,:)=joints(1,:);
        costErrors(counter,1)=errorS;
        costErrors(counter,2)=errorX;
        costErrors(counter,3)=errorY;
        costErrors(counter,4)=cost;
        costErrors(counter,5)=size(joints,1);
        costNames{counter}=fileBrowser.file(j);
        
        counter=counter+1;
        
        %find violations and store them
        violations=findViolations(velper);
        if(~isempty(violations))
            violationSummary(violCounter,1:2)=violations;
            violationSummary(violCounter,3)=size(joints,1);
            violNames{violCounter}=fileBrowser.file(j);
            violCounter=violCounter+1;
        end
    end
end

violNames=violNames';
costNames=costNames';