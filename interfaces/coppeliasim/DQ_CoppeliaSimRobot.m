% (C) Copyright 2011-2025 DQ Robotics Developers
% 
% This file is part of DQ Robotics.
% 
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
% 
% DQ Robotics website: dqrobotics.github.io
% 
% Contributors:
% 
%    1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
%         - Responsible for the original implementation

classdef  (Abstract) DQ_CoppeliaSimRobot < handle
    properties (SetAccess = protected)
        robot_name_;
        coppeliasim_interface_;
    end
    methods(Abstract)
        % This method sets the robot configuration in the CoppeliaSim scene.
        % It is required a dynamics disabled scene. 
        set_configuration(obj);

        % This method sets the target robot configuration in the
        % CoppeliaSim scene. It requires a dynamics-enabled scene
        set_target_configuration(obj);

        % This method returns the robot configuration in the CoppeliaSim scene.
        get_configuration();

        % This method sets the target robot configuration velocities in the CoppeliaSim scene.
        % It requires a dynamics-enabled scene 
        set_target_configuration_velocities(); 

        % This method returns the robot configuration velocities in the CoppeliaSim scene.
        get_configuration_velocities();

        % This method sets the target robot configuration forces in the CoppeliaSim scene.
        % It requires a dynamics-enabled scene.
        set_target_configuration_forces();

        % This method returns the robot configuration forces in the CoppeliaSim scene.
        get_configuration_forces();
    end
end