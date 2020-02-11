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
directory=params.smoothAndPlotAllDir;
period=params.period;
augmentNumPoints=params.augmentNumPoints;
interpNumPoints=params.interpNumPoints;
gaussInterval=params.smoothGaussInterval;

%read the directories (each directory corresponds to a subject)
dirBrowser=DirectoryBrowser(directory);

%loop through each directory
for i=1:dirBrowser.numFolders
    %find files ending in c3d.txt
    fileBrowser=FileBrowser(dirBrowser.folderFullPath(i), '\*c3d.txt');
    for j=1:fileBrowser.numFiles
        disp(fileBrowser.file(j));
        
        %read the file
        [proximal,orientation,~,startEndIndices]=readV3DExport(fileBrowser.fileFullPath(j),period);
        if isempty(startEndIndices)
            disp('Skipping')
            continue;
        end
        %keep just the portion of the activity that we care about
        startEndIndices=modifyStartEndIndices(startEndIndices,fileBrowser.file(j),gaussInterval);

        %truncate the trajectory
        [proximal,orientation]=truncateTrajectory(proximal,orientation,startEndIndices);

        %create frames
        frames=createFrames(proximal,orientation);
        
        close all

        [~,figures]=augmentTrajectory(frames,period,augmentNumPoints,interpNumPoints,gaussInterval,1);

        %linear position
        fig1=figures(1);
        set(fig1,'visible','on');
        fig1.Position(1)=fig1.Position(1)-600;
        fig1.Position(2)=fig1.Position(2)-fig1.Position(4);
        fig1.Position(4)=fig1.Position(4)*2-100;

        %linear velocity
        fig2=figures(2);
        set(fig2,'visible','on');
        fig2.Position(2)=fig2.Position(2)-fig2.Position(4);
        fig2.Position(4)=fig2.Position(4)*2-100;

        %linear acceleration
        fig3=figures(3);
        set(fig3,'visible','on');
        fig3.Position(1)=fig3.Position(1)+600;
        fig3.Position(2)=fig3.Position(2)-fig3.Position(4);
        fig3.Position(4)=fig3.Position(4)*2-100;

        pause;

        %rotation vector
        fig4=figures(4);
        set(fig4,'visible','on');
        fig4.Position(1)=fig4.Position(1)-600;
        fig4.Position(2)=fig4.Position(2)-fig4.Position(4);
        fig4.Position(4)=fig4.Position(4)*2-100;

        %angular velocity
        fig5=figures(5);
        set(fig5,'visible','on');
        fig5.Position(2)=fig5.Position(2)-fig5.Position(4);
        fig5.Position(4)=fig5.Position(4)*2-100;

        %angular acceleration
        fig6=figures(6);
        set(fig6,'visible','on');
        fig6.Position(1)=fig6.Position(1)+600;
        fig6.Position(2)=fig6.Position(2)-fig6.Position(4);
        fig6.Position(4)=fig6.Position(4)*2-100;
        
        pause;
    end
end