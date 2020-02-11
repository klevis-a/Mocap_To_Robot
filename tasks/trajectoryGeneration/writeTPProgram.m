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
tpParameters=TPProgramParams(fullfile(path,'..','..','parameters','tpProgram_humerus.xml'));

%parameters
period=params.period;
multipliers=params.tpProgramMultiplier;
j1=params.tpProgramJ1;
j1offset=params.tpProgramJ1Offset;
jointsFile=params.tpProgramJointsFile;
indicesFile=params.tpProgramIndicesFile;

if contains(indicesFile,'uniform','IgnoreCase',true)
    ssMethod=0;
else
    ssMethod=1;
end

toolframe=robotI.HumerusToolframe;
robot=robotI.robot;
base=robotI.base;
ee=robotI.ee;

timer_id=tpParameters.tpProgramTimer;
cntTag=tpParameters.tpProgramCntTag;
utool=tpParameters.tpProgramUTool;
uframe=tpParameters.tpProgramUFrame;


%read the joints file and indices
[path,name,ext]=fileparts(jointsFile);
joints=readJointsFile(jointsFile);
indices=csvread(indicesFile);

%keep just the joints indicated by the indices
keptJoints=joints(indices,:);

%compute angular difference between positions and multiply it by a
%multiplier to account for the fact that is an average speed but the robot
%thinks that it's a maximum speed
angleDiff=computeAngleDifferences(keptJoints,robot,base,ee,toolframe);

%account for how robot joints are programmed and how matlab and the
%optimization frame account for joints
keptJoints(:,3)=keptJoints(:,3)-keptJoints(:,2);

if j1offset
    keptJoints(:,1)=keptJoints(:,1)+deg2rad(j1);
else    
    keptJoints(:,1)=keptJoints(:,1)+(deg2rad(j1)-keptJoints(1,1));
end

%compute the time
time=(indices-1)*period;
timeDiff=diff(time);

for i=1:length(multipliers)
    currentMult=multipliers(i);
    multiplierId=num2str(round((currentMult-1)*100,0));
    %determine program name
    [subjectId,activity,trialNumber] = fileNameParser(strcat(name,ext));
    if ssMethod==1
        pName = strcat(activity, '_', subjectId, '_', trialNumber, '_', multiplierId,'_N');
    else
        pName = strcat(activity, '_', subjectId, '_', trialNumber, '_', multiplierId,'_U');
    end
    initPName = strcat(pName, '_INIT');
    programPath = fullfile(path,strcat(pName,'.ls'));
    initProgramPath = fullfile(path,strcat(initPName,'.ls'));

    %create program headers for both the motion and init programs
    programHeader(pName, programPath);
    programHeader(initPName, initProgramPath);

    %open both files
    fidM=fopen(programPath,'a');
    fidI=fopen(initProgramPath,'a');
    
    %create the program and starts timers
    fprintf(fidI, '/MN\r\n');
    fprintf(fidI,'   1:  UTOOL_NUM=%s ;\r\n',utool);
    fprintf(fidI,'   2:  UFRAME_NUM=%s ;\r\n',uframe);
    
    fprintf(fidM, '/MN\r\n');
    fprintf(fidM,'   1:  TIMER[%s]=RESET ;\r\n',timer_id);
    fprintf(fidM,'   2:  TIMER[%s]=START ;\r\n',timer_id);
    fprintf(fidM,'   3:  UTOOL_NUM=%s ;\r\n',utool);
    fprintf(fidM,'   4:  UFRAME_NUM=%s ;\r\n',uframe);
    
    %create the motion program
    for n=1:length(indices)
        %create the INIT program
        if n==1
            fprintf(fidI,'   %d:L P[%d] %dmsec FINE   ;\r\n',n+2,n,1000);
        else
            current_pos = indices(n);
            previous_pos = indices(n-1);
            fprintf(fidM,'   %d:L P[%d] %ddeg/sec CNT%s   ;\r\n',n+3,n-1,max(round(currentMult*angleDiff(n-1)/timeDiff(n-1),0),1), cntTag);
        end
    end
    
    %stop the timer
    fprintf(fidM,'   %d:  TIMER[%s]=STOP ;\r\n',4+length(indices),timer_id);

    fprintf(fidM, '/POS\r\n');
    fprintf(fidI, '/POS\r\n');
    
    for n=1:length(indices)
        %print out the position ID
        if n==1
            fprintf(fidI,'P[%d]{\r\n', n);
            fid=fidI;
        else
            fprintf(fidM,'P[%d]{\r\n', n-1);
            fid=fidM;
        end

        %print out the position definition
        fprintf(fid,'   GP1:\r\n');
        fprintf(fid,'\tUF : 0, UT : %s,', utool);
        fprintf(fid,'\tJ1 =   %f  deg,	J2 =  %f  deg,	J3 =  %f  deg,\r\n', rad2deg(keptJoints(n,1)) ,rad2deg(keptJoints(n,2)) ,rad2deg(keptJoints(n,3)));
        fprintf(fid,'\tJ4 =     %f deg,	J5 =   %f deg,	J6 =  %f deg\r\n', rad2deg(keptJoints(n,4)), rad2deg(keptJoints(n,5)), rad2deg(keptJoints(n,6)));
        fprintf(fid,'};\r\n');
    end

    fprintf(fidM, '/END\r\n');
    fprintf(fidI, '/END\r\n');

    fclose(fidI);
    fclose(fidM);
end