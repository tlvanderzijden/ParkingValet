%HelperBehavioralPlanner Behavioral planner helper class.
%   This is a helper class which acts as a simplified behavioral layer in a
%   hierarchical planning workflow. Use the requestManeuver method to
%   request the next maneuver in the route plan.
%
%   NOTE: The name of this class and it's functionality may change without
%   notice in a future release, or the class itself may be removed.
%
%   behaviorPlanner = HelperBehavioralPlanner(costmap, routePlan, vehicleDims, steerLimit) 
%   returns a behavior planner object for the environment specified by
%   vehicleCostmap object costmap, for a route plan specified by table
%   routePlan. vehicleDims is a vehicleDimensions object describing the
%   dimensions of the vehicle, and steerLimit is a scalar specifying the
%   maximum possible steering angle for the vehicle (specified in degrees).
%
%   HelperBehavioralPlanner properties: 
%   Costmap     - Costmap on which route plan is to be executed. 
%   RoutePlan   - Global route plan.
%
%   HelperBehavioralPlanner methods: 
%   requestManeuver     - Request next maneuver along route plan 
%   reachedDestination  - Check if destination has been reached 
%   replanNeeded        - Command to specify that maneuver needs replanning

% Copyright 2017-2018 The MathWorks, Inc.
classdef HelperBehavioralPlanner < handle
    
    properties (SetAccess = protected)
        %Costmap Costmap on which route plan is to be executed.
        %   Costmap is an object of type vehicleCostmap and represents the
        %   map on which route planning is executed.
        Costmap
        
        %RoutePlan Global route plan to be executed.
        %   RoutePlan is a table with variables Start, End and Attributes,
        %   with each row representing a segment of the route plan. Start
        %   and End variables represent the starting and ending poses along
        %   the center of the segment for a particular segment. Attributes
        %   is a struct containing information about the segment -
        %   StopLine, TurnManeuver and SpeedLimit. StopLine specifies
        %   whether the segment ends with a stop line, TurnManeuver
        %   specifies whether the segment represents a turn maneuver, and
        %   SpeedLimit specifies the speed limit in meters/sec.
        RoutePlan
        
        %VehicleDimensions
        %   VehicleDimensions is a vehicleDimensions object containing the
        %   dimensions of the vehicle.
        VehicleDimensions
        
        %SteerLimit
        %   A scalar specifying maximum possible steering angle the vehicle
        %   can achieve, specified in degrees.
        SteerLimit
    end
    
    properties (Access = protected)
        %GoalIndex
        %   Index to next goal
        GoalIndex
        
        %ReplanMode
        %   Flag indicating whether behavioral planner needs to replan.
        ReplanMode = false;
    end
    
    methods
        %------------------------------------------------------------------
        function this = HelperBehavioralPlanner(costmap, routePlan, ...
                vehicleDims, steerLimit)
            
            this.validateCostmap(costmap);
            this.Costmap = costmap;
            
            this.validateRoutePlan(routePlan);
            this.RoutePlan = routePlan;
            
            validateattributes(vehicleDims, {'vehicleDimensions'}, ...
                {'scalar'}, mfilename, 'vehicleDims');
            validateattributes(steerLimit, {'double'}, ...
                {'scalar', '>=', 0, '<=', 90}, mfilename, 'steerLimit');
            
            this.VehicleDimensions = vehicleDims;
            this.SteerLimit        = steerLimit;
            
            reset(this);
        end
        
        %------------------------------------------------------------------
        function [nextGoal, plannerConfig, speedProfile] = requestManeuver(this, currentPose, currentSpeed)
            %requestManeuver Request next maneuver along route plan
            %   [nextGoal, plannerConfig, speedProfile] = requestManeuver(behaviorPlanner, currentPose, currentSpeed)
            %   requests the next maneuver along the route plan. The
            %   maneuver is specified as the nextGoal, a 1-by-3 vehicle
            %   pose [x,y,theta], a plannerConfig struct with planner
            %   settings, and a speedProfile struct with speed profile
            %   settings. Use the plannerConfig to configure the
            %   pathPlannerRRT object. Use the speedProfile to configure
            %   the HelperSpeedProfileGenerator.
            
            % Determine planner settings
            [nextGoal, plannerConfig] = plannerSettings(this, currentPose);
            
            % Determine speed settings
            speedProfile = speedSettings(this, currentSpeed);
            
            % Point to next goal
            this.GoalIndex = this.GoalIndex + 1;
        end
        
        %------------------------------------------------------------------
        function tf = reachedDestination(this)
            %reachedDestination Check if destination has been reached
            %   tf = reachedDestination(behaviorPlanner) returns true if
            %   the destination has been reached (last maneuver has been
            %   executed), and false otherwise.
            
            destinationIndex = height(this.RoutePlan);
            
            tf = (this.GoalIndex > destinationIndex);
        end
        
        %------------------------------------------------------------------
        function replanNeeded(this)
            %replanNeeded command to specify that maneuver needs replanning
            %   replanNeeded(behaviorPlanner) commands the behavioral
            %   planner to replan the most recently requested maneuver.
            %   This configures the planner settings to be more defensive.
            
            % Reset to previous goal
            this.GoalIndex = max(this.GoalIndex - 1, 1);
            
            % Set replan mode
            this.ReplanMode = true;
        end
    end
    
    methods (Access = protected)
        %------------------------------------------------------------------
        function reset(this)
            
            % Point to first goal in route plan
            this.GoalIndex = 1;
        end
        
        %------------------------------------------------------------------
        function [nextGoal, config] = plannerSettings(this, currentPose)
            
            % Determine next goal pose
            idx = this.GoalIndex;
            
            if idx > height(this.RoutePlan)
                error('Route plan has been executed. No more waypoints left.')
            end
            
            nextGoal = this.RoutePlan.EndPose(idx,:);
            
            % Set planner range, minimum iterations and tolerance
            config.ConnectionDistance = 10;
            config.MinIterations      = 10000;
            config.GoalTolerance      = [9, 9, 90];
            
            % If parking maneuver is to be executed, decrease goal
            % tolerance
            isParkManuever = (this.GoalIndex == height(this.RoutePlan));
            if isParkManuever
                config.GoalTolerance = [0.25 0.25 5]; %[0.25 0.25 5];
            end
            
            % Reduce range of planner if current pose are close to the goal
            dist = sqrt(sum((currentPose(1:2) - nextGoal(1:2)).^2));
            closeToGoal = dist < 10;
            if closeToGoal
                config.ConnectionDistance = 5;
            end
            
            % Use a higher turning radius if the route segment is not a
            % turn maneuver
            isTurnManuever = this.RoutePlan.Attributes(idx).TurnManuever;
            if ~isTurnManuever
                config.MinTurningRadius = 20;
            else
                config.MinTurningRadius = 12;
            end
            
            % Handle replanning by adding more iterations to planner
            isReplanManuever = this.ReplanMode;
            if isReplanManuever
                config.MaxIterations = 1.5e4;
                this.ReplanMode = false;
            end
        end
        
        %------------------------------------------------------------------
        function settings = speedSettings(this, currentSpeed)
            
            settings.StartSpeed = currentSpeed;
            
            segmentAttributes = this.RoutePlan.Attributes(this.GoalIndex);
            
            isStopManuever = segmentAttributes.StopLine;
            isParkManuever = (this.GoalIndex == height(this.RoutePlan));
            isTurnManuever = this.RoutePlan.Attributes(this.GoalIndex).TurnManuever;
            
            if isStopManuever
                % If the segment contains a stop line, this is a stop
                % maneuver. End speed must be zero.
                settings.EndSpeed = 0;
                settings.MaxSpeed = segmentAttributes.SpeedLimit;
            elseif isParkManuever
                % If this is the last segment on the route plan, this is a
                % park maneuver. End speed must be zero and maximum speed
                % must be lower.
                settings.EndSpeed = 0;
                settings.MaxSpeed = 5 / 2.23693629; % 5 mph
            else
                settings.EndSpeed = 0.9 * segmentAttributes.SpeedLimit;
                settings.MaxSpeed = segmentAttributes.SpeedLimit;
            end
            
            if isTurnManuever && ~isParkManuever
                % If this is a turn maneuver, reduce the max speed.
                settings.MaxSpeed = 40;%5 / 2.23693629; % 5 mph
                % End speed must be less than or equal to max speed.
                settings.EndSpeed = min(settings.EndSpeed, settings.MaxSpeed);
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Validation
    %----------------------------------------------------------------------
    methods (Access = protected)
        %------------------------------------------------------------------
        function validateCostmap(~, costmap)
            validateattributes(costmap, {'vehicleCostmap'}, {'scalar'}, ...
                mfilename, 'costmap');
        end
        
        %------------------------------------------------------------------
        function validateRoutePlan(this, routePlan)
            
            validateattributes(routePlan, {'table'}, {'ncols', 3}, ...
                mfilename, 'routePlan');
            
            variableNames = routePlan.Properties.VariableNames;
            
            hasVariables = isempty(setdiff(variableNames, {'StartPose', 'EndPose', 'Attributes'}));
            
            if ~hasVariables
                error('routePlan table is missing some variables.');
            end
            
            % Validate starts
            isCollisionFree = all(checkFree(this.Costmap, [routePlan.StartPose; routePlan.EndPose]));
            
            if ~isCollisionFree
                error('Start and End waypoints for all segments of the routePlan must be collision-free.')
            end
        end
    end
end