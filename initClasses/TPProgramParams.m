classdef TPProgramParams < handle
    properties
        tpProgramTimer
        tpProgramUTool
        tpProgramUFrame
        tpProgramCntTag
    end
    
    methods
        function obj = TPProgramParams(file)
            %read the xml initialization file
            params=xml2struct(file);
            
            obj.tpProgramTimer = params.parameters.tpProgramTimer.Text;
            obj.tpProgramUTool = params.parameters.tpProgramUTool.Text;
            obj.tpProgramUFrame = params.parameters.tpProgramUFrame.Text;
            obj.tpProgramCntTag = params.parameters.tpProgramCntTag.Text;
        end
    end
end

