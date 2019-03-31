%HelperKinematicVehicle Simulation of kinematic behavior of vehicle.
%   This is a helper class that manages pose update for a vehicle with
%   kinematics defined by the bicycle model.
%
%   NOTE: The name of this class and it's functionality may change without
%   notice in a future release, or the class itself may be removed.
%
%   vehicle = HelperKinematicVehicle(vehicleDims) returns a kinematics
%   model of vehicle. vehicleDims is a vehicleDimensions object storing
%   vehicle dimensions.
%
%   HelperKinematicVehicle properties:
%   VehicleDimensions - Vehicle dimensions
%   MaxSteeringAngle  - Maximum steering angle of the vehicle
%   Pose              - Current vehicle pose [x, y, theta] (read-only)
%   Velocity          - Current velocity (in meters/second) (read-only)
%   SteeringAngle     - Steering angle (read-only)
%   AccelCmd          - Acceleration command (in meters/second^2) (read-only)
%   DecelCmd          - Deceleration command (in meters/second^2) (read-only)
%   Direction         - Driving direction (read-only)
%
%   HelperKinematicVehicle methods:
%   setPose                     - Set vehicle pose
%   setDirection                - Set driving direction
%   setVelocity                 - Set vehicle velocity
%   setControlCommand           - Set control commands
%   enableLimitedCommandTime    - Enable control command timeout
%   updateKinematics            - Propagate vehicle kinematic model
%
%   See also HelperVehicleSimulator, vehicleDimensions.
%

% Copyright 2017-2018 The MathWorks, Inc.

classdef HelperKinematicVehicle < handle
    
    properties (SetAccess = private)
        %Pose Current vehicle pose [x, y, theta]
        Pose 
        
        %Velocity Current velocity (in meters/second)
        Velocity   

        %Steering angle
        SteeringAngle
        
        %AccelCmd Acceleration command (in meters/second^2)
        AccelCmd
        
        %DecelCmd Deceleration command (in meters/second^2)
        DecelCmd
        
        %Direction Driving direction: 1 for forward motion and -1 for
        %   reversed motion
        Direction
    end
    
    properties
        %VehicleDimensions A vehicleDimensions object
        %   A vehicleDimensions object encapsulating dimensions of the
        %   vehicle. Vehicle dimensions must be defined in the same world
        %   units as the costmap.
        VehicleDimensions
        
        %MaxSteeringAngle Maximum steering angle
        %
        %   Default: 45 (deg)
        MaxSteeringAngle = 45
    end

    properties (Access = private, Dependent)
        %MinimumTurningRadius
        %   Minimum turning radius of vehicle
        MinTurningRadius
        
    end
    
    properties (Access = private)
        %TimeSinceLastCmd Elapsed time since the last velocity command
        %   This counts the time (in second), since the last unique
        %   velocity command was received.
        TimeSinceLastCmd = tic
                
        %MaxCommandTime The maximum time interval between velocity commands
        %   If more time (in seconds) than MaxCommandTime elapses between
        %   unique velocity commands, the vehicle is stopped.
        %   Default: 1 second
        MaxCommandTime = 1
        
        %EnableLimitedCommandTime Enable velocity command timeout.
        EnableLimitedCommandTime = true;
    end
    
    
    %----------------------------------------------------------------------
    % API
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function obj = HelperKinematicVehicle(vehicleDims)
            %HelperKinematicVehicle
            %   obj = HelperKinematicVehicle(vehicleDims) creates a
            %   kinematic vehicle simulator for a vehicle with dimensions
            %   specified in the vehicleDimensions object vehicleDims.
            
            obj.VehicleDimensions = vehicleDims;
        end
        
        %------------------------------------------------------------------
        function setPose(obj, pose)
            %setPose Set the pose of the vehicle. It also set the control
            %   commands to zeros.
                        
            obj.Pose            = reshape(pose,1,3);
            obj.AccelCmd        = 0;
            obj.DecelCmd        = 0;
            obj.SteeringAngle   = 0;
        end
        
        %------------------------------------------------------------------
        function setVelocity(obj, velocity)
            %setVelocity Set velocity of the vehicle
                        
            obj.Velocity        = velocity;
        end
        
        %------------------------------------------------------------------
        function setDirection(obj, direction)
            %setDirection Set driving direction of the vehicle
                        
            obj.Direction       = direction;
        end
        
        %------------------------------------------------------------------
        function enableLimitedCommandTime(obj, enableLimitedCommandTime)
            %enableLimitedCommandTime Enable velocity command timeout.
            %   enableLimitedCommandTime(obj, true) - Enable limited
            %   command time mode, vehicle's velocity is reset to zero if
            %   no new velocity commands arrive in the last MaxCommandTime
            %   seconds.
            %
            %   enableLimitedCommandTime(obj, false) - Disable limited
            %   command time mode.

            obj.EnableLimitedCommandTime = enableLimitedCommandTime;
        end
        
        %------------------------------------------------------------------
        function setControlCommand(obj, ctrlCmd)
            %setControlCommand Set vehicle speed and steering angle
            %
            %   setControlCommand(obj, [accelCmd, decelCmd, steerCmd]) sets 
            %   the control command for the kinematic vehicle.
            
            obj.AccelCmd        = ctrlCmd(1);
            obj.DecelCmd        = ctrlCmd(2);
            obj.SteeringAngle   = ctrlCmd(3);

            obj.TimeSinceLastCmd = tic;
        end
        
        %------------------------------------------------------------------
        function updateKinematics(obj, dt)
            %updateKinematics Propagate the kinematic model of the vehicle
            %   in time by dt.
            
            validateattributes(dt, {'numeric'}, {'nonnan', 'real', ...
                'finite', 'scalar', 'nonempty'}, 'updateKinematics','dt');
            
            % If no velocity command is received within some time, stop 
            % the vehicle.
            if obj.EnableLimitedCommandTime && toc(obj.TimeSinceLastCmd) >...
                    obj.MaxCommandTime
                obj.SteeringAngle   = 0;
                obj.Velocity        = 0;
            end

            % Limit steering angle to maximum allowable value
            delta = obj.SteeringAngle; 
            obj.SteeringAngle = sign(delta) * min(abs(delta), obj.MaxSteeringAngle);
            
            if abs(obj.SteeringAngle) < 1e-5
                obj.SteeringAngle = 0;
            end
            
            % Propagate vehicle state change based on velocities in rear
            % wheel coordinate
            
            % Integrate to obtain speed. Enforce speed not to cross zero.
            speed = abs(obj.Velocity) + dt*(obj.AccelCmd - obj.DecelCmd);
            obj.Velocity = max(speed, 0)*obj.Direction;
            
            delta_theta = obj.Velocity/obj.VehicleDimensions.Wheelbase * sind(obj.SteeringAngle);
            dx = dt*obj.Velocity*cosd(obj.Pose(3) + obj.SteeringAngle) + ...
                 dt*obj.VehicleDimensions.Wheelbase*sind(obj.Pose(3))*delta_theta;
            dy = dt*obj.Velocity*sind(obj.Pose(3) + obj.SteeringAngle) - ...
                 dt*obj.VehicleDimensions.Wheelbase*cosd(obj.Pose(3))*delta_theta;
            dtheta = dt * delta_theta;
            
            % Update vehicle state accordingly
            obj.Pose = obj.Pose + [dx dy rad2deg(dtheta)];
            obj.Pose(3) = mod(obj.Pose(3), 360);        
        end
    end
    
    %----------------------------------------------------------------------
    % Accessors
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function r = get.MinTurningRadius(obj)
            r = obj.VehicleDimensions.Wheelbase / tand(obj.MaxSteeringAngle);
        end
    end
end

