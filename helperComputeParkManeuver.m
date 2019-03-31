function refPath = helperComputeParkManeuver(costmap, currentPose, ...
    goalPose, vehicleDims, maxSteer)
%helperComputeParkManeuver computes a suitable parking maneuver.
%
%   NOTE: The name of this function and it's functionality may change
%   without notice in a future release, or the function itself may be
%   removed.
%
%   refPath = helperComputeParkManeuver(costmap, currentPose, goalPose, vehicleDims, maxSteer)
%   computes a suitable parking maneuver on the costmap, from currentPose
%   to goalPose for a vehicle with dimensions vehicleDims and steering
%   limit at maxSteer. costmap is an object of class vehicleCostmap.
%   currentPose and goalPose are 1x3 vehicle poses specified as [x, y,
%   theta]. vehicleDims is a vehicleDimensions object specifying dimensions
%   of the vehicle and maxSteer is the maximum possible steering angle in
%   degrees. refPath is an object of type driving.Path containing the path
%   to be executed for the parking maneuver.
%
%   Notes
%   -----
%   - This function attempts 2 strategies to find a parking maneuver. The
%     first strategy attempts to find a collision-free Dubins curve to the
%     goal pose using progressively smaller inflation radii, along a series
%     of steering angles. The second strategy attempts to find a path using
%     pathPlannerRRT using progressively smaller inflation radii, along a
%     series of steering angles.
%
%   See also pathPlannerRRT.

% Copyright 2017 The MathWorks, Inc.

% Find smallest possible turning radius
wheelBase = vehicleDims.Wheelbase;
minRadius = wheelBase / tand(maxSteer);

% Copy the costmap
costmap = copy(costmap);

% Propose a range of increasing turning radii
proposalTurningRadii = minRadius : 0.5 : (minRadius+5);

% Propose a range of decreasing inflation radii
proposalInflationRadii = (costmap.InflationRadius-0.5) : -0.2 : 1.1;

% Try finding a collision-free maneuver using Dubins
refPath = dubinsParkManeuver(costmap, currentPose, goalPose, ...
    proposalTurningRadii, proposalInflationRadii);

if ~isempty(refPath)
    return;
end

% Try finding a collision-free maneuver as a path planning query
refPath = rrtParkManeuver(costmap, currentPose, goalPose, ...
    proposalTurningRadii, proposalInflationRadii);

if isempty(refPath)
    warning('Unable to compute suitable park maneuver')
end
end

%--------------------------------------------------------------------------
function refPath = dubinsParkManeuver(costmap, currentPose, goalPose, ...
    proposalTurningRadii, proposalInflationRadii)

% Convert heading to radians
currentPose(3) = deg2rad(currentPose(3));
goalPose(3)    = deg2rad(goalPose(3));

showPaths = false;

bestManeuver = struct.empty();
for ir = 1 : numel(proposalInflationRadii)
    
    if showPaths
        figure
    end
    
    minLength     = inf;
    minPathPoses  = [];
    minRadius     = [];
    
    % Set the costmap inflation radius
    costmap.InflationRadius = proposalInflationRadii(ir);
    
    for tr = 1 : numel(proposalTurningRadii)
        
        % Compute a maneuver using Dubins
        turningRadius = proposalTurningRadii(tr);
        pathPoses = drivingDubinsInterpolate(currentPose, goalPose, inf, 100, turningRadius);
        
        if showPaths
            subplot(4,3,tr)
            plot(costmap)
            hold on, legend off
            helperPlotVehicle(currentPose, vehicleDimensions)
            plot(pathPoses(:,1),pathPoses(:,2), 'LineWidth', 2)
            xlim([30 55])
            ylim([35 50])
        end
        
        % Check if it is collision-free
        try
            isCollisionFree = all(checkFree(costmap, [pathPoses(:,1:2) rad2deg(pathPoses(:,3))]));
        catch
            isCollisionFree = false;
        end
        
        if isCollisionFree
            
            if showPaths
                text(0.1, 0.1, 'Collision Free', 'FontWeight', 'bold', 'Units', 'Normalized')
            end
            pathLength = drivingDubinsDistance(currentPose, goalPose, turningRadius);
            
            if pathLength < minLength
                minLength       = pathLength;
                minPathPoses    = pathPoses;
                minRadius       = turningRadius;
            end
        end
    end
    bestManeuver(ir).PathPoses = minPathPoses;
    bestManeuver(ir).Length    = minLength;
    bestManeuver(ir).Radius    = minRadius;
end

if showPaths
    hold off
end

% Find the maneuver that uses the largest inflation radius
idx = find(arrayfun(@(s)~isempty(s.PathPoses), bestManeuver), 1, 'first');

if ~isempty(idx)
    refPath = createPathFromManeuver(bestManeuver(idx), currentPose, goalPose, minLength);
end
end

%--------------------------------------------------------------------------
function refPath = rrtParkManeuver(costmap, currentPose, goalPose, ...
    proposalRadii, proposalInflationRadii)

refPath = driving.Path.empty();
for ir = 1 : numel(proposalInflationRadii)
    
    % Set the inflation radius
    costmap.InflationRadius = proposalInflationRadii(ir);
    
    % Create a path planner
    rrtPlanner = pathPlannerRRT(costmap, 'GoalTolerance', [0.1 0.1 5], ...
        'ConnectionDistance', 3);
    
    if checkFree(costmap, currentPose) && checkFree(costmap, goalPose)
        
        % Try finding a path at different turning radii
        for tr = 1 : numel(proposalRadii)
            rrtPlanner.MinTurningRadius = proposalRadii(tr);
            refPath = plan(rrtPlanner, currentPose, goalPose);
            
            % If a valid path is found, break out of the loop
            if checkPathValidity(refPath, costmap)
                break;
            end
        end
    end
    
    if ~isempty(refPath) && checkPathValidity(refPath, costmap)
        break;
    end
end
end

%--------------------------------------------------------------------------
function refPath = createPathFromManeuver(maneuver, currentPose, goalPose, minLength)

numSteps            = 100;
connectionDistance  = inf;
turningRadius       = maneuver.Radius;
connMech = matlabshared.planning.internal.DubinsConnectionMechanism;
connMech.TurningRadius      = turningRadius;
connMech.NumSteps           = numSteps;
connMech.ConnectionDistance = connectionDistance;

currentPose(3) = rad2deg(currentPose(3));
goalPose(3)    = rad2deg(goalPose(3));
refPath = driving.Path.create([currentPose;goalPose], connMech, minLength);
end