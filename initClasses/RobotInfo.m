classdef RobotInfo < handle
    properties
        %toolframe
        HumerusToolframe
        %hemisphere toolframe
        HSToolframe
        %velocity limits
        vel_limits
        %urdf file
        urdf
        %robot
        robot
        %base
        base
        %ee
        ee
    end
    
    methods
        function obj = RobotInfo(paramsFile)
            %read the xml initialization file
            params=xml2struct(paramsFile);
            %process humerus toolframe
            x=str2num(params.parameters.HumerusToolframe.x.Text); %#ok<*ST2NM>
            y=str2num(params.parameters.HumerusToolframe.y.Text);
            z=str2num(params.parameters.HumerusToolframe.z.Text);
            w=str2num(params.parameters.HumerusToolframe.w.Text);
            p=str2num(params.parameters.HumerusToolframe.p.Text);
            r=str2num(params.parameters.HumerusToolframe.r.Text);
            obj.HumerusToolframe=ht(eul2rotm(fliplr(deg2rad([w p r]))), [x/1000 y/1000 z/1000]);
            %proces hemisphere toolframe
            hx=str2num(params.parameters.HSToolframe.x.Text);
            hy=str2num(params.parameters.HSToolframe.y.Text);
            hz=str2num(params.parameters.HSToolframe.z.Text);
            hw=str2num(params.parameters.HSToolframe.w.Text);
            hp=str2num(params.parameters.HSToolframe.p.Text);
            hr=str2num(params.parameters.HSToolframe.r.Text);
            obj.HSToolframe=ht(eul2rotm(fliplr(deg2rad([hw hp hr]))), [hx/1000 hy/1000 hz/1000]);
            %process velocity limits
            obj.vel_limits=str2num(params.parameters.velLimits.Text);
            %urdf
            obj.urdf=params.parameters.urdf.Text;
            %robot
            obj.robot=importrobot(obj.urdf);
            %base
            obj.base=params.parameters.base.Text;
            %end effector
            obj.ee=params.parameters.endEffector.Text;
        end
    end
end

