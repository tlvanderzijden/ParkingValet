classdef HelperSpeedProfileGenerator < matlab.System & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
%HelperSpeedProfileGenerator Generate speed profile along the reference path.
%   For a given reference path, HelperSpeedProfileGenerator generates the
%   corresponding reference speed at each point on the path satisfying the
%   maximum acceleration and the maximum deceleration constraints.
%
%   speedProfiler = HelperSpeedProfileGenerator creates a speed profile
%   generator system object, speedProfiler, that generates a speed profile
%   for a given reference path.
% 
%   speedProfiler = HelperSpeedProfileGenerator(Name,Value) creates a
%   speed profile generator object, speedProfiler, with additional options 
%   specified by one or more Name,Value pair arguments:
% 
%   'StartSpeed'            Speed of the vehicle at the beginning of the  
%                           path. This property is only used in MATLAB.
%
%                           Default : 0 (meters/second)
%
%   'EndSpeed'              Speed of the vehicle at the end of the path.
%                           This property is only used in MATLAB.
%
%                           Default:  0 (meters/second)
%   
%   'MaxSpeed'              Maximum speed of the vehicle on the path. 
%                           This property is only used in MATLAB.
%
%                           Default : 5 (meters/second)
% 
%   'MaxAcceleration'       Maximum longitudinal acceleration 
%
%                           Default : 3 (meters/second^2)  
%  
%   'MaxDeceleration'       Maximum longitudinal deceleration 
% 
%                           Default : 6 (meters/second^2)
% 
%   Step method syntax: 
%   speedProfile = step(speedProfiler, pathLengths) generates a speed 
%   profile given the accumulated lengths of the reference path, pathLengths.
% 
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj) and y = obj() are equivalent.   
% 
%   HelperSpeedProfileGenerator properties:
%   StartSpeed              - Speed at the beginning of the path
%   EndSpeed                - Speed at the end of the path
%   MaxSpeed                - Maximum speed on the path
%   MaxAcceleration         - Maximum longitudinal acceleration  
%   MaxDeceleration         - Maximum longitudinal deceleration 
%
%   HelperSpeedProfileGenerator methods:
%   step                 - Compute acceleration and deceleration commands
%   release              - Allow property value changes
%   clone                - Create a copy of the object 
%   isLocked             - Locked status (logical)
%
% See also HelperPathSmoother.

% Copyright 2017-2018 The MathWorks, Inc.
    
    properties
        %StartSpeed Start speed
        %   Speed of the vehicle at the beginning of the path. 
        %   This property is only used in MATLAB.
        %
        %   Default : 0 (meters/second)
        StartSpeed        = 0
        
        %EndSpeed Ending speed
        %   Speed of the vehicle at the end of the path. 
        %   This property is only used in MATLAB.
        %
        %   Default : 0 (meters/second)
        EndSpeed          = 0
        
        %MaxSpeed Maximum speed
        %   Maximum speed of the vehicle on the path. 
        %   This property is only used in MATLAB.
        %
        %   Default : 5 (meters/second)
        MaxSpeed          = 5
        
        %MaxAcceleration Maximum longitudinal acceleration 
        %
        %   Default : 3 (meters/second^2)
        MaxAcceleration   = 3
        
        %MaxDeceleration Maximum longitudinal deceleration 
        %
        %   Default : 6 (meters/second^2)
        MaxDeceleration   = 6
    end
    
    properties(Access = private)
        %LastSpeedProfile Previous output
        %
        LastSpeedProfile
        
        %LastPathLengths Previous path lengths
        %
        LastPathLengths
    end
    
    properties(Access = private, Nontunable, Logical)
        %HasSpeedConfigInput Flag indicating if there is a speedConfig
        %   input. In MATLAB, all the speed configurations are set via
        %   properties while in Simulink they are passed as an input via
        %   the SpeedConfig input port.
        %
        %   Default:          true
        HasSpeedConfigInput = true
    end
    
    %----------------------------------------------------------------------
    % Setter and constructor
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function set.StartSpeed(obj, startSpeed)
            
            validateattributes(startSpeed, {'single', 'double'}, ...
                {'nonnan', 'real', 'nonempty', 'scalar','nonnegative'}, ...
                'startSpeed', mfilename);
            
            obj.StartSpeed = startSpeed;
        end
        
        %------------------------------------------------------------------
        function set.EndSpeed(obj, endSpeed)
            validateattributes(endSpeed, {'single', 'double'}, ...
                {'nonnan', 'real', 'nonempty', 'scalar','nonnegative'}, ...
                'endSpeed', mfilename);
            
            obj.EndSpeed = endSpeed;
        end
        
        %------------------------------------------------------------------
        function set.MaxSpeed(obj, maxSpeed)
            validateattributes(maxSpeed, {'single', 'double'}, ...
                {'nonnan', 'real', 'nonempty', 'scalar','nonnegative'}, ...
                'maxSpeed', mfilename);
            
            obj.MaxSpeed = maxSpeed;
        end
        
        %------------------------------------------------------------------
        function obj = HelperSpeedProfileGenerator(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:}, 'StartSpeed', 'EndSpeed',...
                'MaxSpeed', 'MaxAcceleration', 'MaxDeceleration');
        end
    end
    
    %----------------------------------------------------------------------
    % Main Algorithm
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj, pathLengths, ~)
            % Perform one-time calculations, such as computing constants
            if isSimulinkBlock(obj) % In Simulink
                obj.HasSpeedConfigInput = true;
            else % In MATLAB
                obj.HasSpeedConfigInput = false;
            end
            
            obj.LastPathLengths  = zeros(size(pathLengths), 'like', pathLengths);
            obj.LastSpeedProfile = zeros(size(pathLengths), 'like', pathLengths);
        end
        
        %------------------------------------------------------------------
        function speedProfile = stepImpl(obj, pathLengths, varargin)
            
            if obj.HasSpeedConfigInput
                narginchk(2,3);
                % Retrieve speed information from configuration
                startSpeed = varargin{1}.StartSpeed;
                endSpeed   = varargin{1}.EndSpeed;
                maxSpeed   = varargin{1}.MaxSpeed;
            else
                narginchk(1,2);
                startSpeed = obj.StartSpeed;
                endSpeed   = obj.EndSpeed;
                maxSpeed   = obj.MaxSpeed;
            end
            
            % Check if the path is new. If not, use the previous output.
            if isequal(pathLengths, obj.LastPathLengths)
                speedProfile = obj.LastSpeedProfile;
                return
            end
            
            % Divide the path to segments based on driving direction
            [segStartIndex, segEndIndex, numSegments] = findSegmentBoundaryPointIndex(obj, pathLengths);
            
            % Set the boundary speeds of all the segments
            segStartSpeeds = [startSpeed, zeros(1, numSegments-1)];
            segEndSpeeds   = [zeros(1, numSegments-1), endSpeed];
            
            % Initialize the speed vector with maximum speed
            speedProfile = ones(size(pathLengths, 1), 1)*maxSpeed;
            
            % Generate speed profile for each segment of the path
            for j = 1:numSegments
                segPathLengths = pathLengths(segStartIndex(j):segEndIndex(j));
                maxSpeed = checkReachMaxSpeed(obj, segPathLengths, segStartSpeeds(j), segEndSpeeds(j), maxSpeed);
                
                % Initialize path segment speed vector
                segpeedProfile = ones(size(segPathLengths, 1), 1)*maxSpeed;
                
                % Accelerate phase
                speed_acc = sqrt(2*obj.MaxAcceleration*segPathLengths + segStartSpeeds(j)^2);
                idx_acc = speed_acc <= maxSpeed;
                segpeedProfile(idx_acc) = sqrt(2*obj.MaxAcceleration*segPathLengths(idx_acc)+segStartSpeeds(j)^2);
                
                % Decelerate phase
                speed_dec = sqrt(2*obj.MaxAcceleration*(segPathLengths(end)-segPathLengths)+segEndSpeeds(j)^2);
                idx_dec = speed_dec <= maxSpeed;
                segpeedProfile(idx_dec) = sqrt(2*obj.MaxAcceleration*(segPathLengths(end)-segPathLengths(idx_dec))+segEndSpeeds(j)^2);
                
                % Assign the whole path speed profile
                speedProfile(segStartIndex(j):segEndIndex(j)) = segpeedProfile;
            end
            
            obj.LastPathLengths  = pathLengths;
            obj.LastSpeedProfile = speedProfile;
        end
    end
    
    %----------------------------------------------------------------------
    % Common functions
    %----------------------------------------------------------------------
    methods (Access = protected)
        %------------------------------------------------------------------
        function validateInputsImpl(obj,pathLengths,varargin)
            % Validate inputs to the step method at initialization
            
            validateattributes(pathLengths, {'single', 'double'}, {'nonnan', ...
                'real', 'column', 'finite', '>=', 0, 'nonempty'}, mfilename, 'PathLengths');
            
            if obj.HasSpeedConfigInput && isSimulinkBlock(obj)
               validateattributes(varargin{1}, {'struct'}, ...
                   {'real', 'nonempty'}, mfilename, 'SpeedConfig');
            end
        end
        
        %------------------------------------------------------------------
        function flag = isInactivePropertyImpl(obj,prop)
            % Return false if property is visible based on object
            % configuration, for the command line and System block dialog
            
            if any(strcmp(prop, {'MaxSpeed', 'EndSpeed', 'StartSpeed'}))
                flag = obj.HasSpeedConfigInput && isSimulinkBlock(obj);
            else
                flag = false;
            end
        end
        
        %------------------------------------------------------------------
        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            if obj.HasSpeedConfigInput && isSimulinkBlock(obj) % In Simulink
                num = 2;
            else
                num = 1;
            end
        end
        
        %------------------------------------------------------------------
        function num = getNumOutputsImpl(~)
            % Define total number of outputs for system with optional
            % outputs
            num = 1;
        end
        
        %------------------------------------------------------------------
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);
            
            % Set private and protected properties
            s.LastSpeedProfile = obj.LastSpeedProfile;
            s.LastPathLengths = obj.LastPathLengths;
            s.HasSpeedConfigInput = obj.HasSpeedConfigInput;
        end
        
        %------------------------------------------------------------------
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            % Set private and protected properties
            obj.LastSpeedProfile = s.LastSpeedProfile;
            obj.LastPathLengths  = s.LastPathLengths;
            obj.HasSpeedConfigInput = s.HasSpeedConfigInput;
            
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
    end
    
    %----------------------------------------------------------------------
    % Simulink only
    %----------------------------------------------------------------------
    methods (Access = protected)
        %------------------------------------------------------------------
        function flag = isInputSizeMutableImpl(~,~)
            % Return false if input size cannot change
            % between calls to the System object
            flag = true;
        end
        
        %------------------------------------------------------------------
        function out = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out = propagatedInputDataType(obj,1);
        end
        
        %------------------------------------------------------------------
        function out = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
        end
        
        %------------------------------------------------------------------
        function out = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = false;
        end
        
        %------------------------------------------------------------------
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = ["Helper", "Speed", "Profile", "Generator"];
        end
        
        %------------------------------------------------------------------
        function varargout = getInputNamesImpl(obj)
            % Return input port names for System block
            varargout{1} = 'PathLengths';
            if obj.HasSpeedConfigInput && isSimulinkBlock(obj) % In Simulink
                varargout{2} = 'SpeedConfig';
            end
        end
        
        %------------------------------------------------------------------
        function name = getOutputNamesImpl(~)
            % Return output port names for System block
            name = 'SpeedProfile';
        end
        
        %------------------------------------------------------------------
        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = propagatedInputSize(obj,1);
        end
    end
    
    %----------------------------------------------------------------------
    % Simulink dialog
    %----------------------------------------------------------------------
    methods(Access = protected, Static)
        %------------------------------------------------------------------
        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(...
                'Title', 'Parameters', ...
                'PropertyList', {'StartSpeed', 'EndSpeed', 'MaxSpeed', ...
                'MaxAcceleration', 'MaxDeceleration'});
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
        function maxSpeed = checkReachMaxSpeed(obj, segPathLengths, ...
                startSpeed, endSpeed, maxSpeed)
            %checkReachMaxSpeed Check if the vehicle can reach the maximum
            %   speed given the path length. If not, return the updated
            %   possible maximum speed.
            segTotalLength = segPathLengths(end);
            dis = (maxSpeed^2 - startSpeed^2)/obj.MaxAcceleration + ...
                (maxSpeed^2 - endSpeed^2) /obj.MaxDeceleration;
            
            % If cannot achieve the specified maximum speed, calculate the
            % maximum possible value
            if dis > segTotalLength 
                maxSpeed = sqrt((obj.MaxAcceleration*obj.MaxDeceleration*segTotalLength+...
                    startSpeed^2*obj.MaxDeceleration+endSpeed^2*obj.MaxAcceleration)/...
                    (obj.MaxAcceleration+obj.MaxDeceleration));
            end
        end
        
        %------------------------------------------------------------------
        function [segStartIndex, segEndIndex, numSegments] = findSegmentBoundaryPointIndex(~, pathLengths)
            %findSegmentBoundaryPointIndex Divide the path to segments 
            %based on accumulated path lengths
            
            segStartIndex = find(pathLengths==0); % Starting points have 0 path length
            segEndIndex  = [segStartIndex(2:end)-1; length(pathLengths)];
            numSegments  = length(segStartIndex);
        end
    end
end
