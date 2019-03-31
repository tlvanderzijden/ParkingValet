classdef HelperCubicSplineFit < matlab.System & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
%HelperCubicSplineFit Generate a smooth path by fitting a spline.
%   This system object generates a cubic parametric spline which 
%   goes through all the waypoints specified in the reference path. 
%
%   splineFitter = HelperCubicSplineFit creates a spline path fitting system
%   object, splineFitter, that generates poses and driving directions along 
%   the smoothed path.
%
%   splineFitter = HelperCubicSplineFit(Name,Value) creates a spline path 
%   fitting system object, splineFitter, with additional options specified
%   by one or more Name,Value pair arguments:
%
%   'Poses'                 A N-by-3 matrix representing the poses in the 
%                           planned path.
%
%   'Directions'            A N-by-1 matrix representing the driving
%                           directions corresponding to all the poses along
%                           the planned path.
%   
%   'NumPoints'             A scalar representing number of poses in the
%                           smoothed path. 
%
%   Step method syntax: 
%   [sPoses, sDirection, sPathLengths] = step(splineFitter) returns the  
%   poses, driving directions and accumulated path lengths along the
%   smoothed path.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj) and y = obj() are equivalent. 
%
%   HelperCubicSplineFit properties:
%   Poses                   - Poses in the planned, unsmooth path
%   Directions              - Driving directions corresponding to all the
%                             poses in the planned path
%   NumPoints               - Number of points on the smoothed path
%
%   HelperCubicSplineFit methods:
%   step                    - Smooth the planned path
%   release                 - Allow property value changes
%   clone                   - Create a copy of the object 
%   isLocked                - Locked status (logical)
%
% See also pathPlannerRRT, driving.Path

% References
% ----------
% [1] Lepetic, Marko, Gregor Klancar, Igor Skrjanc, Drago Matko, and Bostjan 
%     Potocnik. "Time optimal path planning considering acceleration limits." 
%     Robotics and Autonomous Systems 45, no. 3-4 (2003): 199-210.  

% Copyright 2017-2018 The MathWorks, Inc.
    
    properties
        %NumPoints Number of points
        %   Number of discrete points in the smooth path
        %
        %   Default: 1000
        NumPoints = 1000

        %Poses Poses in the smoothed path
        Poses
        
        %Directions Driving direction correspond to Poses
        Directions
    end
    
    properties(Access = private)
        % The following two properties are used to transfer reference path 
        % data within the system object. Depending on the environment the 
        % object is executing in, they are assigned either by public
        % properties, Poses and SpeedProfile, in MATLAB, or by the input
        % ports in Simulink. The selection is determined by the 
        % HasStepInputs property.
        
        %PosesInternal
        PosesInternal
        
        %DirectionsInternal
        DirectionsInternal
        
        %X X-coordinates of a path segment
        X
        
        %Y Y-coordinates of a path segment
        Y
        
        %Theta Heading angles of a path segment
        Theta
        
        %T Variable parameterizes X and Y. Its range is [0, 1].
        T
        
        %SegmentDirection A scalar representing the driving direction of a 
        %   path segment
        SegmentDirection
        
        %NumPathSegments Number of segments in a path
        NumPathSegments
        
        %SplineX Piecewise polynomial structure of X spline
        SplineX
        
        %SplineY Piecewise polynomial structure of Y spline
        SplineY
        
        %SplineDx First derivative of X (Piecewise polynomial structure)
        SplineDx
        
        %SplineDy First derivative of Y (Piecewise polynomial structure)
        SplineDy
        
        % The following three properties are used to store the last output.
        
        %LastPosesOutput
        LastPosesOutput
        
        %LastDirectionsOutput
        LastDirectionsOutput
        
        %LastPathLengthsOutput
        LastPathLengthsOutput
        
        %PositionTolerance Tolerance used to obtain unique poses
	    %
	    %   Default: 1e-2 
	    PositionTolerance  =  1e-2 % meters
    end
    
    properties(Access = private, Nontunable, Logical)
        %HasStepInputs Flag indicating if there is inputs to the step method.
        %   In MATLAB, Poses and Directions inputs are set via properties
        %   while in Simulink they are passed as inputs via input ports.
        %
        %   Default:    true
        HasStepInputs = true
    end
    
    %----------------------------------------------------------------------
    % Setter and constructor
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function set.Poses(obj, poses)
            validateattributes(poses, {'single', 'double'}, ...
                {'nonnan', 'real', 'ncols', 3, 'finite', 'nonempty'}, ...
                mfilename, 'Poses');
      
            obj.Poses = poses;
        end
        
        %------------------------------------------------------------------
        function set.Directions(obj, directions)
            validateattributes(directions, {'single', 'double'}, ... 
                {'nonnan', 'real', 'column', 'finite', 'nonempty'}, ...
                mfilename, 'Directions');
            
            obj.Directions = directions;
        end
        
        %------------------------------------------------------------------
        function obj = HelperCubicSplineFit(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:}, 'Poses', 'Directions', ...
                'NumPoints')
        end
    end
    
    %----------------------------------------------------------------------
    % Main algorithm
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj,varargin)
            %setupImpl Perform one-time calculations
            obj.NumPathSegments = 1;
            
            if isSimulinkBlock(obj) % In Simulink
                obj.HasStepInputs = true;
                % Initialize in Simulink to use isequal
                obj.PosesInternal       = nan(size(varargin{1}), 'like', varargin{1});
                obj.DirectionsInternal  = nan(size(varargin{2}), 'like', varargin{2});
            else % In MATLAB
                obj.HasStepInputs = false;
            end
        end
        
        %------------------------------------------------------------------
        function [sPoses, sDirections, sPathLengths] = stepImpl(obj, varargin)
            %stepImpl Implement the main algorithm by fitting a spline to
            %   the raw poses and returning a smooth path.
            
            % In Simulink, Poses and Direction are from input ports. In
            % MATLAB, they are from properties of the object which are
            % inactive in Simulink.
            if obj.HasStepInputs && isSimulinkBlock(obj)
                %Check if the input poses are new. If not, use the previous
                %output.
                if ~isequal(varargin{1}, obj.PosesInternal)
                    obj.PosesInternal      = varargin{1};
                    obj.DirectionsInternal = varargin{2};
                else
                    sPoses       = obj.LastPosesOutput;
                    sDirections  = obj.LastDirectionsOutput;
                    sPathLengths = obj.LastPathLengthsOutput;
                    return
                end
            else
                obj.PosesInternal      = obj.Poses;
                obj.DirectionsInternal = obj.Directions;
            end
            
            % Get unique poses and corresponding directions
	        % Cannot use uniquetol as it is not codegen compatible
	        [poses, directions] = getUniquePoses(obj);
            
            % Divide the path to segments based on driving direction. When
            % a pose is a direction-switching pose, it belongs to both path
            % segments. The segEndIndex of the first path segment will be 
            % equal to the segStartIndex of the second path segment.
            [segStartIndex, segEndIndex, numSegments] = ...
                findSegmentBoundaryPointIndex(obj, directions);
            
            % Use radian in calculation
            poses(:, 3) = deg2rad(poses(:, 3));
            
            % Initialize the output to specify sizes
            sPoses       = zeros(obj.NumPoints, 3);
            sDirections   = zeros(obj.NumPoints, 1);
            sPathLengths = zeros(obj.NumPoints, 1);
            
            % Divide NumPoints into numSegments of segments
            numSegmentPoses = floor(obj.NumPoints/numSegments); 
            
            % Fit each segment and return poses, directions and accumulated
            % path lengths
            for j = 1:numSegments
                obj.X         = poses(segStartIndex(j):segEndIndex(j), 1);
                obj.Y         = poses(segStartIndex(j):segEndIndex(j), 2);
                obj.Theta     = poses(segStartIndex(j):segEndIndex(j), 3);
                
                % Use segEndIndex instead of segStartIndex to determine the 
                % driving direction in order to avoid inconsistency caused 
                % by direction-switching poses
                obj.SegmentDirection = directions(segEndIndex(j));
                
                % Get the parameter of the parametric curve
                computeIndependentVariable(obj);
                
                % Compute the coefficients of the spline and the first 
                % derivative of the spline
                computeCoef(obj);
                
                % Specify number of poses in the segment
                startPoseIndex = 1+(j-1)*numSegmentPoses;
                if j ~= numSegments % Not the last segment
                    endPoseIndex = j*numSegmentPoses;
                else
                    endPoseIndex = obj.NumPoints;
                end
                
                % Generate parameter values of query points
                tQuery = transpose(linspace(0, 1, endPoseIndex-startPoseIndex+1));
                
                % Interpolate and assign
                sPoses      (startPoseIndex: endPoseIndex, :) = interpolate(obj, tQuery);
                sDirections (startPoseIndex: endPoseIndex, 1) = ones(numel(tQuery), 1) * obj.SegmentDirection;
                sPathLengths(startPoseIndex: endPoseIndex, 1) = getPathLengths(obj, tQuery);
            end
            sPoses(:, 3) = rad2deg(sPoses(:, 3));
            
            % Store the output
            obj.LastPosesOutput       = sPoses;
            obj.LastDirectionsOutput  = sDirections;
            obj.LastPathLengthsOutput = sPathLengths;
        end
        
        %------------------------------------------------------------------
        function computeIndependentVariable(obj)
            %computeIndependentVariable Use normalized accumulated chord
            %   arc length as the independent variable for the parametric
            %   curve: X = X(T) and Y = Y(T)
            distances = sqrt((obj.X(2:end) - obj.X(1:end-1)).^2 + ...
                (obj.Y(2:end) - obj.Y(1:end-1)).^2);
            cumLengths = cumsum([0; distances]);
            obj.T = cumLengths/ cumLengths(end);
        end
        
        %------------------------------------------------------------------
        function sPathLengths = getPathLengths(obj, tQuery)
            %getPathLengths Compute cumulative path lengths of query points
            dx = ppval(obj.SplineDx, tQuery);
            dy = ppval(obj.SplineDy, tQuery);
            vel = sqrt(dx.^2 + dy.^2);
            
            sPathLengths = cumtrapz(tQuery, vel);
            sPathLengths = sPathLengths(:);
        end
        
        %------------------------------------------------------------------
        function computeCoef(obj)
            %computePP Compute coefficients of x and y splines
            x     = obj.X;
            y     = obj.Y;
            theta = obj.Theta;
            t     = obj.T;
            
            % Driving directions affects the sign of end constraints
            dir = obj.SegmentDirection;
            
            disStart = sqrt((x(2)-x(1))^2+(y(2)-y(1))^2)/(t(2) - t(1));
            disEnd   = sqrt((x(end)-x(end-1))^2+(y(end)-y(end-1))^2)/(t(end) - t(end-1));
            
            obj.SplineX = spline(t, [disStart*cos(theta(1))*dir; x; disEnd*cos(theta(end))*dir]);
            obj.SplineY = spline(t, [disStart*sin(theta(1))*dir; y; disEnd*sin(theta(end))*dir]);
            
            % Compute coefficients of the derivative of x and y splines
            computeDerivativeCoef(obj, t);
        end
        
        %------------------------------------------------------------------
        function poses = interpolate(obj, tQuery)
            %interpolate Evaluate the spline for query points
            x  = ppval(obj.SplineX,  tQuery);
            y  = ppval(obj.SplineY,  tQuery);
            dx = ppval(obj.SplineDx, tQuery);
            dy = ppval(obj.SplineDy, tQuery);
            
            if obj.SegmentDirection == -1
                theta = mod(atan2(dy, dx) - pi, 2*pi);
            else
                theta = mod(atan2(dy, dx), 2*pi);
            end
            
            poses = [x(:), y(:), theta(:)];
        end
        
        %------------------------------------------------------------------
        function computeDerivativeCoef(obj, t)
            %computeDerivativeCoef Compute coefficients of the derivative
            %   of the spline. Need to use unmkpp to retrieve coefficients
            %   in code generation.
            
            % dx
            [~, xCoefs, xL] = unmkpp(obj.SplineX);
            dxCoefs = xCoefs(:, 1:3).* repmat([3 2 1], xL, 1);
            obj.SplineDx = mkpp(t, dxCoefs);
            
            % dy
            [~, yCoefs, yL] = unmkpp(obj.SplineY);
            dyCoefs = yCoefs(:, 1:3).* repmat([3 2 1], yL, 1);
            obj.SplineDy = mkpp(t, dyCoefs);
        end
    end
    
    %----------------------------------------------------------------------
    % Common methods
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end

        function validatePropertiesImpl(obj)
            % Validate related or interdependent property values
        end
        
        %------------------------------------------------------------------
        function validateInputsImpl(obj, varargin)
            % Validate inputs to the step method at initialization
            if obj.HasStepInputs && isSimulinkBlock(obj)
                validateattributes(varargin{1}, {'single', 'double'}, ...
                    {'nonnan', 'real', 'ncols', 3, 'finite', 'nonempty'}, ...
                    mfilename, 'Poses');
                
                validateattributes(varargin{2}, {'single', 'double'}, ...
                    {'nonnan', 'real', 'column', 'finite', 'nonempty'}, ...
                    mfilename, 'Directions');
            end
        end
        
        %------------------------------------------------------------------
        function flag = isInactivePropertyImpl(obj,prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
            
            if any(strcmp(prop, {'Poses', 'Directions'}))
                flag = obj.HasStepInputs && isSimulinkBlock(obj);
            else
                flag = false;
            end
        end

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
             if obj.HasStepInputs && isSimulinkBlock(obj) % In Simulink
                 num = 2;
             else
                 num = 0;
             end
        end

        function num = getNumOutputsImpl(~)
            % Define total number of outputs for system with optional
            % outputs
            num = 3;
        end
        
        %------------------------------------------------------------------
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);
            
            % Set private and protected properties
            s.X                     = obj.X;
            s.Y                     = obj.Y;
            s.Theta                 = obj.Theta;
            s.T                     = obj.T;
            s.SegmentDirection      = obj.SegmentDirection;
            s.NumPathSegments       = obj.NumPathSegments;
            s.SplineX               = obj.SplineX;
            s.SplineY               = obj.SplineY;
            s.SplineDx              = obj.SplineDx;
            s.SplineDy              = obj.SplineDy;
            s.PosesInternal         = obj.PosesInternal;
            s.DirectionsInternal    = obj.DirectionsInternal;
            s.LastPosesOutput       = obj.LastPosesOutput;
            s.LastDirectionsOutput  = obj.LastDirectionsOutput;
            s.LastPathLengthsOutput = obj.LastPathLengthsOutput;
        end
        
        %------------------------------------------------------------------
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            % Set private and protected properties
            obj.X                     = s.X;
            obj.Y                     = s.Y;
            obj.Theta                 = s.Theta;
            obj.T                     = s.T;
            obj.SegmentDirection      = s.SegmentDirection;
            obj.NumPathSegments       = s.NumPathSegments;
            obj.SplineX               = s.SplineX;
            obj.SplineY               = s.SplineY;
            obj.SplineDx              = s.SplineDx;
            obj.SplineDy              = s.SplineDy;
            obj.PosesInternal         = s.PosesInternal;
            obj.DirectionsInternal    = s.DirectionsInternal;
            obj.LastPosesOutput       = s.LastPosesOutput;
            obj.LastDirectionsOutput  = s.LastDirectionsOutput;
            obj.LastPathLengthsOutput = s.LastPathLengthsOutput;
            
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
    end
    
    %----------------------------------------------------------------------
    % Simulink-only methods
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function [out1,out2,out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            refPosesSize = propagatedInputSize(obj,1);
            out1 = [refPosesSize(1)*1000 3];
            out2 = [refPosesSize(1)*1000 1];
            out3 = [refPosesSize(1)*1000 1];
        end
        
        %------------------------------------------------------------------
        function [out1,out2,out3] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = propagatedInputDataType(obj,1);
            out2 = propagatedInputDataType(obj,2);
            out3 = propagatedInputDataType(obj,1);
        end
        
        %------------------------------------------------------------------
        function [out1,out2,out3] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
        end
        
        %------------------------------------------------------------------
        function [out1,out2,out3] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out1 = false;
            out2 = false;
            out3 = false;
        end
        
        %------------------------------------------------------------------
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = ["Helper", "Cubic", "Spline", "Fit"]; 
        end
        
        %------------------------------------------------------------------
        function [name,name2] = getInputNamesImpl(~)
            % Return input port names for System block
            name = 'Poses';
            name2 = 'Directions';
        end
        
        %------------------------------------------------------------------
        function [name,name2,name3] = getOutputNamesImpl(~)
            % Return output port names for System block
            name = 'Poses';
            name2 = 'Directions';
            name3 = 'PathLengths';
        end
    end
    
    %----------------------------------------------------------------------
    % Simulink dialog
    %---------------------------------------------------------------------- 
    methods(Access = protected, Static)        
        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(...
                'Title', 'Parameters', ...
                'PropertyList', {'NumPoints'});
        end
    end
    
    %----------------------------------------------------------------------
    % Utility functions
    %----------------------------------------------------------------------
    methods (Access = protected)
        %------------------------------------------------------------------
        function flag = isSimulinkBlock(obj)
            %isSimulinkBlock Check if the system object in used in Simulink
            flag = getExecPlatformIndex(obj); % 0 for MATLAB, 1 for Simulink
        end
        
        %------------------------------------------------------------------
        function [poses, directions] = getUniquePoses(obj)
            %getUniquePoses If two poses are too close, remove one of them
            distances = sqrt((obj.PosesInternal(2:end,1) - obj.PosesInternal(1:end-1,1)).^2 + ...
                (obj.PosesInternal(2:end,2) - obj.PosesInternal(1:end-1,2)).^2);
            rowIndex = [1; find(distances > obj.PositionTolerance) + 1];
            poses      = obj.PosesInternal(rowIndex, :);
            directions = obj.DirectionsInternal(rowIndex, :);
        end
        
        %------------------------------------------------------------------
        function [segStartIndex, segEndIndex, numSegments] = findSegmentBoundaryPointIndex(~, directions)
            %findSegmentBoundaryPointIndex Divide the path to segments 
            %based on driving direction
            
            % Find the switching points
            switchIndex = find(directions(1:end-1)+directions(2:end)==0);
            % Divide the path into segments
            segStartIndex = [1; switchIndex];
            segEndIndex   = [switchIndex; length(directions)];
            numSegments   = length(segStartIndex);
        end
    end
end
