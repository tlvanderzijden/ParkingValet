%pathPlannerRRT Configure an RRT* path planner.
%   pathPlannerRRT configures a path planner based on the Optimal Rapidly
%   Exploring Random Tree (RRT*) algorithm. An RRT planner explores the
%   environment around the vehicle by constructing a tree of random
%   collision-free poses from a specified start pose towards a desired goal
%   region. Once the pathPlannerRRT is configured, use the plan method to
%   plan a path.
%
%   planner = pathPlannerRRT(costmap) returns a pathPlannerRRT object for
%   planning. costmap is a vehicleCostmap object specifying the environment
%   around the vehicle as a costmap.
%
%   planner = pathPlannerRRT(...,Name,Value) specifies additional
%   name-value pair arguments as described below:
%
%
%   'GoalTolerance'         Acceptable tolerance around goal pose specified
%                           as [xTol, yTol, thetaTol]. xTol and yTol are
%                           specified in world units. thetaTol is specified
%                           in degrees.
%
%                           Default: [0.5, 0.5, 5]
%
%   'GoalBias'              Probability of selecting the goal pose at each
%                           iteration as opposed to a random pose. Larger
%                           values accelerate reaching the goal at the risk
%                           of failing to circumnavigate obstacles.
%
%                           Default: 0.1
%
%   'ConnectionMethod'      Method used to calculate connection between
%                           consecutive poses, specified as one of the
%                           strings 'Dubins' or 'Reeds-Shepp'. These
%                           methods compute a kinematically feasible path
%                           between poses as a sequence of primitive
%                           motions.
%
%       -------------------------------------------------------------------
%       Connection Method  | Description
%       -------------------|-----------------------------------------------
%       'Dubins'           | A sequence of 3 primitive motions, each of
%                          | of which can be:
%                          | - straight (forward)
%                          | - left turn at maximum steer (forward)
%                          | - right turn at maximum steer (forward)
%       -------------------|-----------------------------------------------
%       'Reeds-Shepp'      | A sequence of up to 5 primitive motions, each
%                          | of which can be:
%                          | - straight (forward and reverse)
%                          | - left turn at maximum steer (forward and
%                          |   reverse)
%                          | - right turn at maximum steer (forward and
%                          |   reverse)
%       -------------------------------------------------------------------
%
%                           Default: 'Dubins'
%
%   'ConnectionDistance'    Maximum distance between two connected poses.
%                           Distance is computed along the path. Larger
%                           values result in longer path segments between
%                           poses.
%
%                           Default: 5
%
%   'MinTurningRadius'      Minimum turning radius of vehicle specified in
%                           world units. This corresponds to the radius of
%                           the turning circle at maximum steer. A larger
%                           value limits the maximum steering angle used by
%                           the planner.
%
%                           Default: 4 (this is computed using a wheelbase
%                           of 2.8 meters and a maximum steering angle of
%                           35 degrees.)
%
%   'MinIterations'         Minimum number of planner iterations. This is
%                           the minimum number of iterations of exploration
%                           the planner undertakes before returning a path.
%                           
%
%                           Default: 100
%
%   'MaxIterations'         Maximum number of planner iterations. This is
%                           the maximum number of iterations of exploration
%                           the planner undertakes in search of a path. If
%                           no path is found at the end of MaxIterations
%                           number of iterations, the plan method will
%                           return a path with no poses. 
%
%                           Default: 1e4
%
%   'ApproximateSearch'     Flag indicating whether approximate nearest
%                           neighbor search is used. Setting this to true
%                           uses a faster, but approximate search
%                           algorithm. Setting this to false uses an exact,
%                           but slower search algorithm at the cost of
%                           increased computation time.
%
%                           Default: true
%
%
%   pathPlannerRRT properties:
%   Costmap                 - Costmap used for collision checking (read-only)
%   GoalTolerance           - Acceptable tolerance around goal pose
%   GoalBias                - Probability of selecting goal pose
%   ConnectionMethod        - Method used to connect poses
%   ConnectionDistance      - Maximum distance between two connected poses
%   MinTurningRadius        - Minimum turning radius of vehicle
%   MinIterations           - Minimum number of iterations of exploration
%   MaxIterations           - Maximum number of iterations of exploration
%   ApproximateSearch       - Flag indicating use of approximate search
%
%   pathPlannerRRT methods:
%   plan                    - plan a path between start and goal poses
%   plot                    - plot a planned path on a map.
%
%
%   Notes
%   -----
%   - The pathPlannerRRT checks for collisions by verifying that the
%     center of the vehicle is not within an inflated grid cell in the 
%     vehicleCostmap. If the InflationRadius of the vehicleCostmap is set
%     to a value less than the radius of the smallest circle completely
%     encircling the vehicle (default), the planned path may contain
%     collisions.
%   - Updating any of the properties of the planner will clear the planned
%     path from the pathPlannerRRT. Subsequent calls to plot will only show
%     the map.
%   - In order to improve performance, the pathPlannerRRT uses an 
%     approximate nearest neighbor search that only searches through
%     sqrt(N) nodes, where N is the number of nodes to search. This can
%     result in less optimal paths. Set 'ApproximateSearch' to false to use
%     exact nearest neighbor search.
%   - 'Dubins' and 'Reeds-Shepp' connection methods are kinematically
%     feasible and ignore inertial effects. This makes the planner suitable
%     for low velocity environments where the inertial effects of wheel
%     forces are small.
%
%
%   Example: Plan a path towards parking spot
%   -----------------------------------------
%   % Load a costmap for a parking lot
%   data = load('parkingLotCostmap.mat');
%   parkMap = data.parkingLotCostmap;
%
%   % Define a start and goal pose as [x,y,theta]
%   startPose = [4, 4, 90];   % [meters, meters, degrees]
%   goalPose  = [30, 13, 0];
%
%   % Create an RRT planner
%   planner = pathPlannerRRT(parkMap);
%
%   % Plan a route from start to goal pose
%   refPath = plan(planner, startPose, goalPose);
%
%   % Plot planned route
%   plot(planner)
%
%
%   See also vehicleCostmap, checkPathValidity, driving.Path.

% Copyright 2017-2018 The MathWorks, Inc.

%   References
%   ----------
%   [1] S. Karaman, E. Frazzoli, "Optimal kinodynamic motion planning using
%       incremental sampling-based methods," IEEE Conference on Decision
%       and Control (CDC), pp. 7681-7687, 2010.
%
%   [2] A. Shkel, V. Lumelsky, "Classification of the Dubins set," Robotics
%       and Autonomous Systems, vol. 34, issue 4, pp. 179-202, 2001.
%
%   [3] J.A. Reeds, L.A. Shepp, "Optimal paths for a car that goes both
%       forwards and backwards," Pacific Journal of Mathematics,
%       145(2):3670393, 1990.

classdef pathPlannerRRT < matlabshared.planning.internal.PathPlanner
    
    properties (SetAccess = private)
        %Costmap Costmap used for collision checking.
        %   Costmap is an object of type vehicleCostmap and represents the
        %   map around the vehicle to be used for collision checking during
        %   planning.
        Costmap
    end
    
    properties (Dependent)
        %ConnectionMethod Method used to connect poses.
        %   Method used to connect two nodes (poses) specified as one of
        %   the strings 'Dubins' or 'Reeds-Shepp'.
        %
        %   ---------------------------------------------------------------
        %   Connection Method  | Description
        %   -------------------|-------------------------------------------
        %                      |
        %   'Dubins'           | Connects two poses by computing a
        %                      | kinematically feasible path between them
        %                      | as a sequence of 3 primitive motions, each
        %                      | of which can be:
        %                      | - Straight (forward)
        %                      | - Left turn at maximum steer (forward)
        %                      | - Right turn at maximum steer (forward)
        %                      |
        %   -------------------|-------------------------------------------
        %   'Reeds-Shepp'      | Connects two poses by computing a
        %                      | kinematically feasible path between them
        %                      | as a sequence of up to 5 primitive
        %                      | motions, each of which can be:
        %                      | - Straight (forward and reverse)
        %                      | - Left turn at maximum steer (forward and
        %                      |   reverse)
        %                      | - Right turn at maximum steer (forward and
        %                      |   reverse)
        %                      |
        %   ---------------------------------------------------------------
        %
        %   Default: 'Dubins'
        ConnectionMethod
        
        %ConnectionDistance Maximum distance between two connected nodes.
        %   Maximum distance between two connected nodes. Distance is
        %   computed along the path.
        %
        %   Default: 5
        ConnectionDistance
        
        %MinTurningRadius Minimum turning radius of vehicle.
        %   Minimum turning radius of vehicle specified in world units.
        %   This corresponds to the radius of the turning circle at maximum
        %   steer. A larger value limits the maximum steering angle used by
        %   the planner.
        %
        %   Default: 4
        MinTurningRadius
        
        %GoalTolerance Tolerance around goal pose.
        %   Acceptable tolerance around goal pose specified as a 3-element
        %   vector [xTol, yTol, thetaTol]. xTol and yTol are specified in
        %   world units. thetaTol is specified in degrees. A goal is
        %   reached if the planner finds a path to a pose within [+/-xTol,
        %   +/-yTol, +/-thetaTol] of the goalPose.
        %
        %   Default: [0.5, 0.5, 5]
        GoalTolerance
        
        %GoalBias Probability of selecting goal pose.
        %   Probability with which to select goal pose for tree expansion.
        %   Increasing this value biases exploration towards goal at the
        %   cost of possibly getting stuck at an obstacle.
        %
        %   Default: 0.1
        GoalBias
                
        %MinIterations Minimum number of iterations of exploration.
        %   Minimum number of iterations of exploration the planner
        %   undertakes before returning a path.
        %
        %   Default: 100
        MinIterations
        
        %MaxIterations Maximum number of iterations of exploration.
        %   Maximum number of iterations of exploration the planner
        %   undertakes in search of a path. If no path is found at the end
        %   of MaxIterations number of iterations, the plan method will
        %   return a path with no poses.
        %
        %   Default: 1e4
        MaxIterations
    end
    
    properties
        %ApproximateSearch Flag indicating use of approximate search.
        %   Flag indicating whether approximate nearest neighbor search is
        %   used. True or false.
        %
        %   Default: true
        ApproximateSearch = true;
    end
    
    properties (Access = protected)
        Name    = mfilename;
        
        PoseDim = 3;
    end
    
    properties (Access = ?tpathPlannerRRT, Hidden)
        InternalPlanner = matlabshared.planning.internal.RRTPlanner;
    end
    
    properties (Access = private, Hidden)
        Version = ver('driving');
        
        Path
    end
    
    properties (Access = private, Dependent, Transient)
        %NumConnectionSteps
        %   Number of steps along connection.
        NumConnectionSteps
    end
    
    properties (Access = private, Transient)
        %PlanAttempted
        %   Flag indicating whether planning was attempted.
        PlanAttempted = false;
    end
    
    %----------------------------------------------------------------------
    % Interface
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = pathPlannerRRT(varargin)
            
            plannerDims = 3;
            this = this@matlabshared.planning.internal.PathPlanner(mfilename, plannerDims);
            
            inputs = this.parseInputs(varargin{:});
            
            % Make a copy of the map
            this.Costmap = copy(inputs.Costmap);
            
            % Convert theta tolerance to radians before passing to internal
            % planner
            inputs.GoalTolerance(3) = deg2rad(inputs.GoalTolerance(3));
            this.InternalPlanner.GoalTolerance = double(inputs.GoalTolerance);
            
            % Set directly-dependent properties
            this.InternalPlanner.GoalBias      = double(inputs.GoalBias);
            this.InternalPlanner.MinIterations = double(inputs.MinIterations);
            this.InternalPlanner.MaxIterations = double(inputs.MaxIterations);
            
            this.ApproximateSearch = inputs.ApproximateSearch;
            
            this.configure(inputs);
        end
        
        %------------------------------------------------------------------
        function varargout = plan(this, startPose, goalPose)
            %plan Plan a path using RRT*.
            %   refPath = plan(planner, startPose, goalPose) plans a path
            %   from startPose towards goalPose using RRT*.
            %
            %   [refPath,tree] = plan(...) additionally returns the
            %   exploration tree.
            %
            %
            %   Inputs
            %   ------
            %   startPose   Initial vehicle pose specified as [x,y,theta].
            %               Specify x, y in world units and theta in
            %               degrees.
            %
            %   goalPose    Goal vehicle pose specified as [x,y,theta].
            %               Specify x, y in world units and theta in
            %               degrees.
            %
            %   Outputs
            %   -------
            %   refPath     Planned vehicle path returned as a
            %               <a href="matlab:help('driving.Path')">driving.Path</a> object, composed of a series of
            %               path segments. If planning is not successful
            %               and a path is not found, the path has no
            %               segments. Use the <a href="matlab:help('checkPathValidity')">checkPathValidity</a> function to
            %               check if the path is still valid in case the
            %               costmap is updated. Use the <a href="matlab:help('driving.Path/interpolate')">interpolate</a> method
            %               to interpolate poses along the path.
            %
            %   tree        Explored tree returned as a <a href="matlab:help('digraph')">digraph</a> object
            %               with nodes representing explored vehicle poses
            %               and edges representing distance between
            %               connected nodes.
            %
            %
            %   Example : Plan a path
            %   ---------------------
            %   % Load a costmap for a parking lot
            %   data = load('parkingLotCostmap.mat');
            %   parkMap = data.parkingLotCostmap;
            % 
            %   % Define a start and goal pose as [x,y,theta]
            %   startPose = [ 4,  4, 90];   % [meters, meters, degrees]
            %   goalPose  = [15, 38,  0];
            %
            %   % Create an RRT planner
            %   connDistance  = 10;  % Maximum distance between RRT nodes
            %   minIterations = 1000; 
            %   planner = pathPlannerRRT(parkMap, 'ConnectionDistance', ...
            %       connDistance, 'MinIterations', minIterations);
            %
            %   % Plan a path to the goal
            %   refPath = plan(planner, startPose, goalPose)
            %
            %   % Check if a path was found.
            %   pathFound = ~isempty(refPath.PathSegments)
            %
            %   % Plot the planner with exploration tree
            %   figure, plot(planner, 'Tree', 'on')
            %
            %   See also checkPathValidity, digraph.
            
            nargoutchk(0,2);
            
            this.validatePoses(startPose, goalPose);
            
            startPose(3) = matlabshared.planning.internal.angleUtilities.convertAndWrapTo2Pi(startPose(3));
            goalPose(3)  = matlabshared.planning.internal.angleUtilities.convertAndWrapTo2Pi(goalPose(3));
            
            this.Path = driving.Path.create();
            pathPoses = this.InternalPlanner.planPath(startPose, goalPose);
            
            % Convert theta back to degrees
            pathPoses(:,3) = rad2deg(pathPoses(:,3));
            
            this.Path = this.createPath(pathPoses);
            this.PlanAttempted = true;
            
            varargout{1} = this.Path;
            
            if nargout>1
                varargout{2} = this.InternalPlanner.Tree.toDigraph();
            end
        end
        
        %------------------------------------------------------------------
        function plot(this, varargin)
            %plot Plot a planned path.
            %
            %   plot(planner) plots the path planned by the pathPlannerRRT
            %   planner.
            %
            %   plot(...,Name,Value) specifies additional name-value pair
            %   arguments as described below:
            %
            %   'Parent'    Handle to an axes on which to display the path.
            %
            %   'Tree'      A string 'on' or 'off' to turn on or off the
            %               display of poses explored by RRT*.
            %
            %               Default: 'off'
            %
            %   'Vehicle'   A string 'on' or 'off' to turn on or off the
            %               display of the vehicle along the found path.
            %
            %               Default: 'on'
            %
            %   Notes
            %   -----
            %   If a path has not been found, a path has not been planned
            %   or properties of the planner have changed, only the costmap
            %   will be displayed.
            %
            %   See also vehicleCostmap.
            
            params = this.parsePlotInputs(varargin{:});
            
            % Prepare axes handle according to NextPlot
            hAx = newplot(params.Parent);
            
            % Display costmap
            plot(this.Costmap, 'Parent', hAx, 'Inflation', 'on');
            
            % Record current hold state
            if ishold(hAx)
                oldState = 'on';
            else
                oldState = 'off';
            end
            
            % Turn hold on
            hold(hAx, 'on')
            
            % Restore hold state on completion
            restoreHoldState = onCleanup(@()hold(hAx, oldState));
            
            % Change resolution factor for faster processing of plot
            resolutionFactor = 5;
            this.InternalPlanner.updateConnectionSteps( ceil(this.NumConnectionSteps / resolutionFactor) );
            
            % Restore resolution factor on completion
            restoreResolution = onCleanup(@()restoreNumSteps(this));
            
            function restoreNumSteps(this)
                this.InternalPlanner.updateConnectionSteps( this.NumConnectionSteps );
            end
            
            % Define colors
            colors.Tree  = [0.9290    0.6940    0.1250];
            colors.Path  = [     0    0.4470    0.7410];
            colors.Goal  = [0.8500    0.3250    0.0980];%[0.4660    0.6740    0.1880];%[0.6350    0.0780    0.1840];
            
            
            % Plot exploration tree
            if strcmp(params.Tree, 'on')
                plotTree(this, hAx, colors.Tree);
            end
            
            % Plot planned path
            plotPath(this, hAx, colors.Path, params.Vehicle);
            
            % Plot start and goal poses
            plotEndPoints(this, hAx, colors.Goal);
            
            % Turn on legend if there is a path
            if ~isempty(this.InternalPlanner.GoalPose)
                legend;
            end
        end
    end
    
    
    %----------------------------------------------------------------------
    % Configuration
    %----------------------------------------------------------------------
    methods (Access = private)
        %------------------------------------------------------------------
        function configure(this, inputs)
            
            % Configure internal planner
            this.InternalPlanner.configure(this.Costmap, ...
                inputs.ConnectionMethod, inputs.ConnectionDistance, ...
                inputs.MinTurningRadius, inputs.ApproximateSearch);
            
            % Set connection interpolation resolution
            this.InternalPlanner.updateConnectionSteps(this.NumConnectionSteps);
        end
        
        %------------------------------------------------------------------
        function refPath = createPath(this, pathPoses)
            
            if strcmpi(this.ConnectionMethod, 'Dubins')
                
                % Create a Dubins connection object
                conn = driving.internal.planning.DubinsConnection.create();
                conn.MinTurningRadius = this.MinTurningRadius;
                
                % Create an array of Dubins path segments
                pathSeg = driving.DubinsPathSegment.empty();
                for n = 1 : size(pathPoses,1)-1
                    pathSeg(n) = driving.DubinsPathSegment.create(...
                        conn, pathPoses(n,:), pathPoses(n+1,:));
                end
            else
                
                % Create a Reeds-Shepp connection object
                conn = driving.internal.planning.ReedsSheppConnection.create();
                conn.MinTurningRadius = this.MinTurningRadius;
                
                % Create an array of Reeds-Shepp path segments
                pathSeg = driving.ReedsSheppPathSegment.empty();
                for n = 1 : size(pathPoses,1)-1
                    pathSeg(n) = driving.ReedsSheppPathSegment.create(...
                        conn, pathPoses(n,:), pathPoses(n+1,:));
                end
            end
            
            % Create a path object to hold the array of path segments
            refPath = driving.Path.create(pathSeg);
        end
    end
    
    
    %----------------------------------------------------------------------
    % Accessors
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function set.Costmap(this, costmap)
            
            this.validateCostmap(costmap);
            this.Costmap = costmap;
            
            this.PlanAttempted = false; %#ok<MCSUP>
        end
        
        %------------------------------------------------------------------
        function set.GoalTolerance(this, gtol)
            
            this.validateGoalTolerance(gtol);
            gtol(3) = matlabshared.planning.internal.angleUtilities.convertAndWrapTo2Pi(gtol(3));
            this.InternalPlanner.GoalTolerance = gtol;
            
            this.PlanAttempted = false;
        end
        
        %------------------------------------------------------------------
        function gtol = get.GoalTolerance(this)
            
            gtol = this.InternalPlanner.GoalTolerance;
            gtol(3) = rad2deg(gtol(3));
        end
        
        %------------------------------------------------------------------
        function set.GoalBias(this, gbias)
            
            this.validateGoalBias(gbias);
            this.InternalPlanner.GoalBias = gbias;
            
            this.PlanAttempted = false;
        end
        
        %------------------------------------------------------------------
        function gbias = get.GoalBias(this)
            
            gbias = this.InternalPlanner.GoalBias;
        end
        
        %------------------------------------------------------------------
        function set.ConnectionMethod(this, cmethod)
            
            this.InternalPlanner.updateConnectionMethod( ...
                matlabshared.planning.internal.validation.checkConnectionMethod( ...
                cmethod, mfilename));
            
            this.PlanAttempted = false;
        end
        
        %------------------------------------------------------------------
        function cmethod = get.ConnectionMethod(this)
            
            cmethod = this.InternalPlanner.ConnectionMechanism.Name;
        end
        
        %------------------------------------------------------------------
        function set.ConnectionDistance(this, cdist)
            
            this.validateConnectionDistance(cdist);
            this.InternalPlanner.updateConnectionDistance( double(cdist) );
            this.InternalPlanner.updateConnectionSteps(this.NumConnectionSteps);
            
            this.PlanAttempted = false;
        end
        
        %------------------------------------------------------------------
        function cdist = get.ConnectionDistance(this)
            
            cdist = this.InternalPlanner.ConnectionMechanism.ConnectionDistance;
        end
        
        %------------------------------------------------------------------
        function set.MinTurningRadius(this, radius)
            
            this.validateMinTurningRadius(radius);
            this.InternalPlanner.updateTurningRadius( double(radius) );
            
            this.PlanAttempted = false;
        end
        
        %------------------------------------------------------------------
        function radius = get.MinTurningRadius(this)
            
            radius = this.InternalPlanner.ConnectionMechanism.TurningRadius;
        end
        
        %------------------------------------------------------------------
        function set.MinIterations(this, minIter)
            
            this.validateIterations(minIter, 'MinIterations');
            this.checkIterations(minIter, this.MaxIterations);
            
            this.InternalPlanner.MinIterations = double(minIter);
            
            this.PlanAttempted = false;
        end
        
        %------------------------------------------------------------------
        function minIter = get.MinIterations(this)
            
            minIter = this.InternalPlanner.MinIterations;
        end
        
        %------------------------------------------------------------------
        function set.MaxIterations(this, maxIter)
            
            this.validateIterations(maxIter, 'MaxIterations');
            this.checkIterations(this.MinIterations, maxIter);
            
            this.InternalPlanner.MaxIterations = double(maxIter);
            
            this.PlanAttempted = false;
        end
        
        %------------------------------------------------------------------
        function maxIter = get.MaxIterations(this)
            
            maxIter = this.InternalPlanner.MaxIterations;
        end
        
        %------------------------------------------------------------------
        function set.ApproximateSearch(this, approxSearch)
            
            this.validateApproxSearch(approxSearch);
            this.ApproximateSearch = logical(approxSearch);
            this.InternalPlanner.updateApproxSearch( logical(approxSearch) ); %#ok<MCSUP>
            
            this.PlanAttempted = false; %#ok<MCSUP>
        end
        
        %------------------------------------------------------------------
        function numSteps = get.NumConnectionSteps(this)
            
            % Number of connection steps is used to control the resolution
            % at which collision checking is performed against the map. We
            % use a resolution factor of 5, i.e. an interpolation step at a
            % distance of 0.2*cellSize.
            resolutionFactor = 5;
            
            connDist = this.ConnectionDistance;
            if isfinite(connDist)
                numSteps = resolutionFactor * max(3, ...
                    ceil(connDist/this.Costmap.CellSize) );
            else
                numSteps = resolutionFactor * norm( size(this.Costmap.Costmap) );
            end
        end
    end
    
    
    %----------------------------------------------------------------------
    % Save/Load
    %----------------------------------------------------------------------
    methods (Static, Hidden)
        %------------------------------------------------------------------
        function this = loadobj(that)
            
            this = pathPlannerRRT(that.Costmap, ...
                'GoalTolerance',        that.GoalTolerance, ...
                'GoalBias',             that.GoalBias, ...
                'ConnectionMethod',     that.ConnectionMethod, ...
                'ConnectionDistance',   that.ConnectionDistance, ...
                'MinTurningRadius',     that.MinTurningRadius, ...
                'MinIterations',        that.MinIterations, ...
                'MaxIterations',        that.MaxIterations, ...
                'ApproximateSearch',    that.ApproximateSearch);
        end
    end
    
    methods (Hidden)
        %------------------------------------------------------------------
        function that = saveobj(this)
            
            that.Costmap            = this.Costmap;        % this is an object
            that.GoalTolerance      = this.GoalTolerance;
            that.GoalBias           = this.GoalBias;
            that.ConnectionMethod   = this.ConnectionMethod;
            that.ConnectionDistance = this.ConnectionDistance;
            that.MinTurningRadius   = this.MinTurningRadius;
            that.MinIterations      = this.MinIterations;
            that.MaxIterations      = this.MaxIterations;
            that.ApproximateSearch  = this.ApproximateSearch;
            that.Version            = this.Version;
        end
    end
    
    
    %----------------------------------------------------------------------
    % Input Parsing
    %----------------------------------------------------------------------
    methods (Access = private)
        %------------------------------------------------------------------
        function inputs = parseInputs(this, varargin)
            
            parser = inputParser;
            parser.FunctionName = mfilename;
            
            parser.addRequired('Costmap', @(cm)validateattributes(cm,{'vehicleCostmap'},{'nonempty'},mfilename));
            parser.addParameter('GoalTolerance', [0.5 0.5 5], @this.validateGoalTolerance);
            parser.addParameter('GoalBias', 0.1, @this.validateGoalBias);
            parser.addParameter('ConnectionMethod', 'Dubins');
            parser.addParameter('ConnectionDistance', 5, @this.validateConnectionDistance);
            parser.addParameter('MinTurningRadius', 4, @this.validateMinTurningRadius);
            parser.addParameter('MinIterations', 100, @(minIter)this.validateIterations(minIter, 'MinIterations'));
            parser.addParameter('MaxIterations', 1e4, @(maxIter)this.validateIterations(maxIter, 'MaxIterations'));
            parser.addParameter('ApproximateSearch', true, @this.validateApproxSearch);
            
            parser.parse(varargin{:});
            
            inputs = parser.Results;
            
            % Check if iterations make sense
            this.checkIterations(inputs.MinIterations, inputs.MaxIterations);
            
            % Adjust connection method char to be complete
            inputs.ConnectionMethod = matlabshared.planning.internal.validation.checkConnectionMethod(...
                inputs.ConnectionMethod, mfilename);
        end
        
        %------------------------------------------------------------------
        function validatePoses(this, startPose, goalPose)
            
            this.validatePose(startPose, 'startPose');
            this.validatePose(goalPose, 'goalPose');
            
            % Here poses are still in degrees.
            poses = [startPose;goalPose];
            
            % Convert to radians.
            poses(:,3) = deg2rad(poses(:,3));
            
            % Check that poses are obstacle-free.
            throwError = false;
            free = this.Costmap.checkFreePoses(poses, throwError);
            
            if any(~free)
                if ~free(1)
                    error(message('shared_autonomous:pathPlannerRRT:posesInCollision', 'startPose'))
                end
                if ~free(2)
                    %error(message('shared_autonomous:pathPlannerRRT:posesInCollision', 'goalPose'))
                end
            end
        end
        
        %------------------------------------------------------------------
        function inputs = parsePlotInputs(this, varargin)
            
            parser = inputParser;
            parser.FunctionName = mfilename;
            
            parser.addParameter('Parent', [], @this.validateParent);
            parser.addParameter('Tree', 'off', @this.validateOnOff);
            parser.addParameter('Vehicle', 'on', @this.validateOnOff);
            
            parser.parse(varargin{:});
            
            inputs = parser.Results;
        end
        
        %------------------------------------------------------------------
        function tf = validateParent(~, parent)
            
            tf = driving.internal.validation.checkParent(parent);
        end
        
        %------------------------------------------------------------------
        function validateOnOff(~, onoff)
            
            % This will error if it's not an expected on/off value.
            matlab.lang.OnOffSwitchState(onoff);
        end
        
        %------------------------------------------------------------------
        function checkIterations(~, minIter, maxIter)
            
            validateattributes(minIter, {'numeric'}, ...
                {'real','nonsparse','scalar','<=',maxIter}, mfilename, 'MinIterations');
        end
        
        %------------------------------------------------------------------
        function validateGoalBias(~, gbias)
            validateattributes(gbias, {'single','double'}, ...
                {'real','nonsparse','scalar','>=', 0, '<', 1}, mfilename, 'GoalBias');
        end
        
        %------------------------------------------------------------------
        function validateIterations(~, iter, iterName)
            validateattributes(iter, {'numeric'}, ...
                {'real', 'nonsparse', 'scalar', 'positive', 'finite', 'integer'}, mfilename, ...
                iterName);
        end
        
        %------------------------------------------------------------------
        function validateMinTurningRadius(~, radius)
            validateattributes(radius, {'single','double'}, ...
                {'real','nonsparse','scalar', 'positive', 'finite'}, mfilename, ...
                'MinTurningRadius');
        end
        
        %------------------------------------------------------------------
        function validateApproxSearch(~, approxSearch)
            validateattributes(approxSearch, {'numeric','logical'}, ...
                {'real','nonsparse','scalar', 'binary'}, mfilename, 'ApproximateSearch');
        end
    end
    
    
    %----------------------------------------------------------------------
    % Plotting
    %----------------------------------------------------------------------
    methods (Access = private)
        %------------------------------------------------------------------
        function plotTree(this, hAx, treeColor)
            
            if ~this.PlanAttempted
                return;
            end
            
            treeNodes = this.InternalPlanner.Tree.Nodes;
            treeEdges = this.InternalPlanner.Tree.Edges;
            
            if isempty(treeNodes) || isempty(treeEdges)
                return;
            end
            
            switch this.ConnectionMethod
                case 'Dubins'
                    numTransitions = 2;
                case 'Reeds-Shepp'
                    numTransitions = 4;
            end
            
            % Plot poses using scatter
            scatter(hAx, treeNodes(:,1), treeNodes(:,2), [], treeColor, ...
                'filled', 'Tag', 'rrtExploredNodes', ...
                'DisplayName', ...
                getString(message('shared_autonomous:pathPlannerRRT:treeLegendString')));
            
            numSteps = this.InternalPlanner.ConnectionMechanism.NumSteps;
            numEdges = size(treeEdges,1);
            
            % Compute all interpolated poses
            poses = nan((numSteps+2)*numEdges, 3);
            idx = 1 : numSteps;
            for n = 1 : numEdges
                
                from = treeEdges(n,1);
                to   = treeEdges(n,2);
                
                % Add start of link
                poses(idx(1),:) = treeNodes(from,:);
                
                % Add points along link
                interpPoses = this.InternalPlanner.ConnectionMechanism.interpolate(...
                    treeNodes(from,:), treeNodes(to,:));
                
                poses(idx+1,:) = interpPoses(numTransitions+1:end,:);
                
                idx = idx + numSteps + 2;
            end
            
            hLines = plot(hAx, poses(:,1), poses(:,2), 'Color', treeColor, ...
                'LineWidth', 1, 'Tag', 'rrtExploredPath');
            
            % Turn off legend display of lines
            hLines.Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
        
        %------------------------------------------------------------------
        function plotPath(this, hAx, pathColor, vehicleOnOff)
            
            path = this.Path;
            
            if ~this.PlanAttempted
                return;
            end
            
            if isempty(this.Path) || isempty(this.Path.PathSegments)
                
                % If a plan was attempted but a path was not found, add a
                % legend entry showing that the path was not found.
                if this.PlanAttempted
                    
                    % Compose a string stating that the path was not found.
                    notFoundName = strcat(...
                        getString(message('shared_autonomous:pathPlannerRRT:pathLegendString')), ...
                        [' ', getString(message('shared_autonomous:pathPlannerRRT:notFoundString'))]);
                    
                    % Add a scatter plot with NaNs to display the legend
                    scatter(hAx, NaN, NaN, [], pathColor, 'filled', ...
                        'DisplayName', notFoundName);
                end
                return;
            end
            
            vdims       = this.Costmap.VehicleDimensions;
            legendName  = getString(message('shared_autonomous:pathPlannerRRT:pathLegendString'));
            
            plot(path, 'Parent', hAx, 'Color', pathColor, ...
                'Vehicle', vehicleOnOff, 'VehicleDimensions', vdims, ...
                'DisplayName', legendName, 'Tag', 'rrtSolved');
        end
        
        %------------------------------------------------------------------
        function plotEndPoints(this, hAx, goalColor)
            
            goalPose  = this.InternalPlanner.GoalPose;
            if ~isempty(goalPose)
                scatter(hAx, goalPose(:,1), goalPose(:,2), [], goalColor, ...
                    'filled', 'Tag', 'rrtGoal', 'DisplayName', ...
                    getString(message('shared_autonomous:pathPlannerRRT:goalLegendString')));
            end
        end
    end
end