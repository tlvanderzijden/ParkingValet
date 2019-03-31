%% Automated Parking Valet
% This example shows how to construct an automated parking valet system. 
%
% Copyright 2017-2018 The MathWorks, Inc.

clear all
clc
close all
simulateDrive = 1; %do we want to simulate the calculated paths

%% Initialisation of costmap
% represent occupied cells, and light cells represent free cells.
mapLayers = loadParkingLotMapLayers; %upload maplayers 
mapLayers = rmfield(mapLayers,'ParkedCars'); 

% For simplicity, combine the three layers into a single costmap.
costmap = combineMapLayers(mapLayers);
vehicleWidth = 2; %vehicle width rounded to 0.5 metres
vehicleHeight = 4; %vehicle height
costVal = 1; %cost value when cell is occupied
load('carActivation.mat'); %load which parking spaces are occupied
load('carPosition.mat'); %load the coordinates and angle of the parking spots

%the costvalue of grid cells of occupied parking spots is set to 1 in the costmap
numberOfCars = 44;  
numberOfParkedCars = size(carActivation(carActivation>0),1); %number of parking spaces in parking lot
sizeCarMesh = (vehicleWidth/costmap.CellSize+1)*(vehicleHeight/costmap.CellSize+1); 
xyPoints = zeros(numberOfParkedCars*sizeCarMesh,2);
for i = 1:numberOfCars
   if carActivation(i)>0
       %set car cost
       [x,y] = meshgrid(carPosition(i,1):costmap.CellSize:carPosition(i,1)+ vehicleWidth, ...
           carPosition(i,2):costmap.CellSize:carPosition(i,2)+vehicleHeight);
       xyPoints((i-1)*(sizeCarMesh)+1: (i-1)*(sizeCarMesh)+ sizeCarMesh,:) = [x(:), y(:)]; 
   end
   setCosts(costmap,xyPoints,costVal);
end

%%
% The |costmap| covers the entire 75m-by-50m parking lot area, divided into
% 0.5m-by-0.5m square cells.
costmap.MapExtent ;% [x, y, width, height] in meters
costmap.CellSize   ; % cell size in meters

%% Initialization of the vehicle
% Create a vehicle object for
% storing the dimensions of the vehicle that will park automatically. Also
% define the maximum steering angle of the vehicle. This value determines
% the limits on the turning radius durickcg motion planning and control.
vehicleDims      = vehicleDimensions;
maxSteeringAngle = 35; % in degrees
costmap.VehicleDimensions = vehicleDims; % update the costmap with the vehicledimensions to park
costmap.CollisionChecker.NumCircles = 4; %initialize a collisionchecker

%% The parking spot which is closest to the supermarket is chosen as the goal position
load('parkPoses.mat');
marketPos = [75 34];
distanceToMarket = sqrt((marketPos(1)-parkPoses(:,1)).^2+(marketPos(2)-parkPoses(:,2)).^2); 
Occupied = carActivation;
carIndex = [1:numberOfCars]';
parkMap = table(carIndex, parkPoses, distanceToMarket, Occupied); 
parkMap = sortrows(parkMap, 'distanceToMarket'); 
parkMap.Preference = [1:numberOfCars]';
posChoosen = false; 
i = 0;
while ~posChoosen
    i = i + 1; 
    if ~parkMap.Occupied(i)
        parkPose = parkMap.parkPoses(i,:); 
        posChoosen = true; 
        disp(['The nearest parking spot from the supermarket is spot ', num2str(parkMap.carIndex(i))]); 
    end
end

%determine the routeplan to the parking spot
StartPose = [4 12 0];
EndPose = parkPose; 
Attributes.SpeedLimit = 2; 
Attributes.StopLine = 1;
Attributes.TurnManuever = 1; 
routePlan = table(StartPose, EndPose, Attributes);
routePlan(end+1,:) = routePlan(end,:);
%% Routemap
% Plot costmap and a vehicle at the current pose, and along each goal in the routenplan.
figure
plot(costmap, 'Inflation', 'on')
legend off

hold on
helperPlotVehicle(routePlan.StartPose(1,:), vehicleDims, 'DisplayName', 'Current Pose')
legend

%plot the routeplan
for n = 1 : height(routePlan)
    % Extract the goal waypoint
    vehiclePose = routePlan{n, 'EndPose'};
    
    % Plot the pose
    legendEntry = sprintf('Goal %i', n);
    helperPlotVehicle(vehiclePose, vehicleDims, 'DisplayName', legendEntry);
end
hold off

%% Behavioral planner
% Create the behavioral planner helper object. The |requestManeuver| method
% requests a stream of navigation tasks from the behavioral planner until
% the destination is reached.
 
behavioralPlanner = HelperBehavioralPlanner(costmap, routePlan, ...
    vehicleDims, maxSteeringAngle);
behavioralPlanner.Costmap.CollisionChecker.NumCircles = 1; 

%% Speed initialization
maxSpeed = 20; %maximum speed to start of parking maneuver
startSpeed = 0; % in meters/second
endSpeed   = 0; % in meters/second

%% Create a speedProfileGenerator object to generate a speed profile based on the smoothed path.
speedProfileGenerator = HelperSpeedProfileGenerator(startSpeed, endSpeed, maxSpeed);

%% Initialisation vehicle simulation
% Create the vehicle simulator
vehicleSim = HelperVehicleSimulator(costmap, vehicleDims);
sim = gcf;

% Set the vehicle pose and velocity 
vehicleSim.setVehiclePose(routePlan.StartPose(1,:));
currentVel = 0;
vehicleSim.setVehicleVelocity(currentVel);

% Configure the simulator to show the trajectory
vehicleSim.showTrajectory(true);

%% Create a HelperLongitudinalController object to control the velocity of the vehicle and specify the sample time.
sampleTime = 0.05;
lonController = HelperLongitudinalController('SampleTime', sampleTime);

%% Use the HelperFixedRate object to ensure fixed-rate execution of the feedback controller. 
%Use a control rate to be consistent with the longitudinal controller.
controlRate = HelperFixedRate(1/sampleTime); % in Hertz

%% Execute a Simulation
%In the simulation first a path is found from the start position to a
%pre-park position. This is done by performing RRT* to the park position
%with a big goal tolerance. One a path to the pre park position is found
%it is smoothened and driving of the car towards the goal is simulated. When the car reaches the pre park
%position RRT settings are adjusted to make a smooth backwards parking
%maneuver possible. When a path is found from the pre park position to the
%park position it is again smoothened and a simulation towards the parking
%position takes place.

% Set the vehicle pose back to the initial starting point
currentPose = routePlan.StartPose(1,:); % [x, y, theta]
vehicleSim.setVehiclePose(currentPose);

% Reset velocity
currentVel  = 0; % meters/second
vehicleSim.setVehicleVelocity(currentVel);

% Configure the simulator to show the trajectory
vehicleSim.showTrajectory(true);
currentManeuver = 1; 
     
while ~reachedDestination(behavioralPlanner)
    % Reset velocity
    currentVel  = 0; % meters/second
    vehicleSim.setVehicleVelocity(currentVel);
    
    % Configure the simulator to show the trajectory
    vehicleSim.showTrajectory(true);
    % Request next maneuver from behavioral layer
    [nextGoal, plannerConfig, speedConfig] = requestManeuver(behavioralPlanner, ...
        currentPose, currentVel);
    
    % Configure the motion planner dependent on maneuver
    if currentManeuver == height(routePlan) %parking
        motionPlanner = pathPlannerRRT(costmap, 'MinIterations', 10000, ...
            'ConnectionMethod', 'Reeds-Shepp');
        costmap.CollisionChecker.NumCircles = 4; 
        plannerConfig.MinTurningRadius = 10; 
        plannerConfig.ConnectionDistance = 15; 
        
        figure
        plot(costmap, 'Inflation', 'on')
        legend off
    else %no parking
        motionPlanner = pathPlannerRRT(costmap, 'MinIterations', 10000, ...
            'ConnectionDistance', 10, 'MinTurningRadius', 20, 'GoalTolerance', [9 9 45], 'ConnectionMethod' , 'Dubins');
        costmap.CollisionChecker.NumCircles = 4;     
    end
    configurePlanner(motionPlanner, plannerConfig);
    
    % Plan a reference path using RRT* planner to the next goal pose
    refPath = plan(motionPlanner, currentPose, nextGoal);
    routePlan.EndPose(1,:) = refPath.GoalPose;
    nextGoal = routePlan.EndPose(1,:);
    routePlan.StartPose(2,:) = refPath.GoalPose; 
    
    % Check if the path is valid. If the planner fails to compute a path,
    % or the path is not collision-free because of updates to the map, the
    % system needs to re-plan. This scenario uses a static map, so the path
    % will always be collision-free.
    isReplanNeeded = ~checkPathValidity(refPath, costmap);
    if isReplanNeeded
        warning('Unable to find a valid path. Attempting to re-plan.')
        
        % Request behavioral planner to re-plan
        replanNeeded(behavioralPlanner);
        %continue;
    end
    
    % Configure the motion planner
    if currentManeuver == height(routePlan)
        numSamples = 10;
        stepSize   = refPath.Length / numSamples;
        lengths    = 0 : stepSize : refPath.Length;
        [transitionPoses, directions] = interpolate(refPath, lengths);
    else
        [transitionPoses, directions] = interpolate(refPath);
    end
    
    % % Create an object to smooth the path using spline fitting
    splineFitter = HelperCubicSplineFit(transitionPoses, directions);
    % Number of spline points along the path
    splineFitter.NumPoints = 1000;
    % Assign poses and directions to the spline fitting object
    splineFitter.Poses      = transitionPoses;
    splineFitter.Directions = directions;
    % Fit the spline
    [refPoses, directions, refPathLengths] = splineFitter();
    
    % Visualize the planned path
    if currentManeuver == height(routePlan)
        figure
        plotParkingManeuver(costmap, refPath, currentPose, nextGoal)
        figure
        plot(motionPlanner, 'Tree','on')
        
        % Plot the smoothed path
        hold on
        hSmoothPath = plot(refPoses(:, 1), refPoses(:, 2), 'r', 'LineWidth', 2, ...
            'DisplayName', 'Smoothed Path');
        hold off
    else
        figure
        plot(motionPlanner, 'Tree','on')
        
        % Plot the smoothed path
        hold on
        hSmoothPath = plot(refPoses(:, 1), refPoses(:, 2), 'r', 'LineWidth', 2, ...
            'DisplayName', 'Smoothed Path');
        hold off
    end
    
    % Generate a trajectory using the speed profile generator
    configureSpeedProfileGenerator(speedProfileGenerator, speedConfig);
    
    % Generate a speed profile based on the accumulated lengths along the 
    refSpeeds = speedProfileGenerator(refPathLengths);
    maxSpeed = speedConfig.MaxSpeed;
    %plotSpeedProfile(refPathLengths, refSpeeds, directions, maxSpeed)
    pathAnalyzer = HelperPathAnalyzer(refPoses, refSpeeds, directions, ...
    'Wheelbase', vehicleDims.Wheelbase);

    % Configure path analyzer
    pathAnalyzer.RefPoses     = refPoses;
    pathAnalyzer.Directions   = directions;
    pathAnalyzer.SpeedProfile = refSpeeds;
    
    %add goalposition to routeplan
    routePlan.StartPose(end, :) = refPath.GoalPose;
    nextGoal = refPath.GoalPose; 
    routePlan.EndPose(end,:) = parkPose;
    
    % Reset longitudinal controller 
    reset(lonController);
    reachGoal = false; 
    if simulateDrive == 1
        while ~reachGoal
            % Get current driving direction
            currentDir = getDrivingDirection(vehicleSim);
            % Find the reference pose on the path and the corresponding velocity
            [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
            
            %If the vehicle changes driving direction, reset vehicle velocity in
            %the simulator
            if currentDir ~= direction
                currentVel = 0;
                setVehicleVelocity(vehicleSim, currentVel);
            end
            
            
            % Update driving direction for the simulator
            updateDrivingDirection(vehicleSim, direction);
            
            % Compute steering command
            steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
                'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
            
            % Compute acceleration and deceleration commands
            lonController.Direction = direction;
            [accelCmd, decelCmd] = lonController(refVel, currentVel);
            
            % Simulate the vehicle using the controller outputs
            drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
            
            % Check if the vehicle reaches the goal
            reachGoal = helperGoalChecker(nextGoal, currentPose, currentVel, endSpeed, direction);
            waitfor(controlRate);
            
            % Get current pose and velocity of the vehicle
            currentPose  = getVehiclePose(vehicleSim);
            currentVel   = getVehicleVelocity(vehicleSim);
         
        end
        % Show vehicle simulation figure
        showFigure(vehicleSim);
    else
        currentPose  = routePlan.EndPose(currentManeuver,:);
    end
currentManeuver = currentManeuver + 1; 
end %while reached destination

%% Supporting Functions
%%%
% *loadParkingLotMapLayers*
% Load environment map layers for parking lot
function mapLayers = loadParkingLotMapLayers()
%loadParkingLotMapLayers
%   Load occupancy maps corresponding to 3 layers - obstacles, road
%   markings, and used spots.

mapLayers.StationaryObstacles = imread('stationary.bmp');
mapLayers.RoadMarkings        = imread('road_markings.bmp');
mapLayers.ParkedCars          = imread('parked_cars.bmp');
end

%%%
% *plotMapLayers*
% Plot struct containing map layers
function plotMapLayers(mapLayers)
%plotMapLayers
%   Plot the multiple map layers on a figure window.

figure
cellOfMaps = cellfun(@imcomplement, struct2cell(mapLayers), 'UniformOutput', false);
montage( cellOfMaps, 'Size', [1 numel(cellOfMaps)], 'Border', [5 5], 'ThumbnailSize', [300 NaN] )
title('Map Layers - Stationary Obstacles, Road markings, and Parked Cars')
end

%%%
% *combineMapLayers*
% Combine map layers into a single costmap
function costmap = combineMapLayers(mapLayers)
%combineMapLayers
%   Combine map layers struct into a single vehicleCostmap.

combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings ;
combinedMap = im2single(combinedMap);

res = 0.5; % meters
costmap = vehicleCostmap(combinedMap, 'CellSize', res);
end

%%%
% *configurePlanner*
% Configure path planner with specified settings
function configurePlanner(pathPlanner, config)
%configurePlanner
% Configure the path planner object, pathPlanner, with settings specified
% in struct config.

fieldNames = fields(config);
for n = 1 : numel(fieldNames)
    pathPlanner.(fieldNames{n}) = config.(fieldNames{n});
end
end

%%%
% *configureSpeedProfileGenerator*
% Configure speed profile generator with specified settings
function configureSpeedProfileGenerator(speedProfiler, config)
%configureSpeedProfileGenerator
% Configure speed profiler, speedProfiler, with settings specified in
% struct config.

fieldNames = fields(config);
for n = 1 : numel(fieldNames)
    speedProfiler.(fieldNames{n}) = config.(fieldNames{n});
end
end

%%%
% *plotSpeedProfile*
% Plot speed profile
function plotSpeedProfile(refPathLengths, refSpeeds, directions, maxSpeed)
figure
%plotSpeedProfile
% Plot the generated speed profile

% Plot reference speeds along length of the path
cumPath = refPathLengths;
for i = 2:length(refPathLengths)
    if refPathLengths(i) == 0
        cumPath(i:end) = cumPath(i:end)+refPathLengths(i-1);
    end
end
refSpeeds = directions.*refSpeeds;
plot(cumPath, refSpeeds, 'LineWidth', 2);

% Plot a line to display maximum speed
hold on
line([0;cumPath(end)], [maxSpeed;maxSpeed], 'Color', 'r')
hold off

% Set axes limits
buffer = 2;
xlim([0 cumPath(end)]);
ylim([min(refSpeeds)*1.1 max(refSpeeds)*1.1])

% Add labels
xlabel('Path Length (m)');
ylabel('Speed (m/s)');

% Add legend and title
legend('Speed Profile', 'Max Speed')
title('Generated speed profile')
end

%%%
% *plotParkingManeuver*
% Display the generated parking maneuver on a costmap
function plotParkingManeuver(costmap, refPath, currentPose, parkPose)
%plotParkingManeuver
% Plot the generated parking maneuver on a costmap.
% Plot the costmap, without inflated areas
plot(costmap, 'Inflation', 'off')

% Plot reference parking maneuver on the costmap
hold on
plot(refPath, 'DisplayName', 'Parking Maneuver')

title('Parking Maneuver')

% Zoom into parking maneuver by setting axes limits
lo = min([currentPose(1:2); parkPose(1:2)]);
hi = max([currentPose(1:2); parkPose(1:2)]);

buffer = 20; % meters

xlim([lo(1)-buffer hi(1)+buffer])
ylim([lo(2)-buffer hi(2)+buffer])
end