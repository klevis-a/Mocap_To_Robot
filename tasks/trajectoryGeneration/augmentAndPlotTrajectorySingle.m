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
c3dfile=params.smoothAndPlotFile;
period=params.period;
augmentNumPoints=params.augmentNumPoints;
interpNumPoints=params.interpNumPoints;
gaussInterval=params.smoothGaussInterval;

%read the file
[proximal,orientation,~,startEndIndices]=readV3DExport(c3dfile,period);
if isempty(startEndIndices)
    disp('Skipping')
    return;
end
%keep just the portion of the activity that we care about
startEndIndices=modifyStartEndIndices(startEndIndices,c3dfile,gaussInterval);

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