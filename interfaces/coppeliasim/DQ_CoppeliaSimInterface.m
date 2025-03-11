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
%         - Responsible for the original implementation, which is based on
%           https://github.com/dqrobotics/cpp-interface-coppeliasim/blob/main/include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h

classdef  (Abstract) DQ_CoppeliaSimInterface < handle
    methods(Abstract)
            % This method connects to CoppeliaSim.
            % Calling this function is required before anything else can happen.
            status = connect(obj, host, port, timeout_in_milliseconds);
    
            % This method enables or disables the stepped (synchronous) mode
            % for the remote API server service to which the client is connected to.
            % Example:
            %       set_stepping_mode(true)    % stepping mode enabled
            %       set_stepping_mode(false)   % stepping mode disabled
            set_stepping_mode(obj, flag);
    
            % This method sends trigger signal to the CoppeliaSim scene, 
            % which performs a simulation step when the stepping mode is used.
            trigger_next_simulation_step(obj);
    
            % This method starts the CoppeliaSim simulation.
            start_simulation(obj);
    
            % This method stops the CoppeliaSim simulation.
            stop_simulation(obj);
    
            % This method gets the handles for a cell array of 
            % object names in the the CoppeliaSim scene.
            handles = get_object_handles(obj, objectnames);
    
            % This method gets the handle for a given object in the CoppeliaSim scene. 
            handle = get_object_handle(obj, objectname);
    
            % This method gets the translation of an object in the CoppeliaSim scene.
            t = get_object_translation(obj, objectname);
    
            % This method sets the translation of an object in the CoppeliaSim scene.
            set_object_translation(obj, objectname, translation);
    
            % This method gets the rotation of an object in the CoppeliaSim scene.
            r = get_object_rotation(obj, objectname);
    
            % This method sets the rotation of an object in the CoppeliaSim scene.
            set_object_rotation(obj, objectname, rotation);
    
            % This method gets the pose of an object in the CoppeliaSim scene. 
            x = get_object_pose(obj, objectname);
    
            % This method sets the pose of an object in the CoppeliaSim scene. 
            set_object_pose(obj, objectname, pose);
    
            % This method sets the joint positions in the CoppeliaSim scene.        
            set_joint_positions(obj, jointnames, joint_positions);
    
            % This method gets the joint positions in the CoppeliaSim scene. 
            joint_positions = get_joint_positions(obj, jointnames);
    
            % This method sets the joint target positions in the CoppeliaSim scene. 
            % It requires a dynamics-enabled scene and joints in dynamic mode with position control mode. 
            % For more information about joint modes:
            % https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
            set_joint_target_positions(obj, jointnames, joint_target_positions);
    
            % This method gets the joint velocities in the CoppeliaSim scene. 
            joint_velocities = get_joint_velocities(obj, jointnames);
    
            % This method sets the joint target velocities in the CoppeliaSim scene.
            % It requires a dynamics-enabled scene and joints in dynamic mode with velocity control mode. 
            % For more information about joint modes:
            % https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
            set_joint_target_velocities(obj, jointnames, joint_target_velocities);
    
            % This method sets the joint target forces in the CoppeliaSim scene.
            % It requires a dynamics-enabled scene and joints in dynamic mode with force control mode. 
            % For more information about joint modes:
            % https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
            set_joint_target_forces(obj, jointnames, torques);  
    
            % This method gets the joint forces in the CoppeliaSim scene.
            joint_torques = get_joint_forces(obj, jointnames);
      end
end

