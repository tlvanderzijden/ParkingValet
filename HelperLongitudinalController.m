classdef HelperLongitudinalController < matlab.System & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.SampleTime & matlab.system.mixin.CustomIcon 
%HelperLongitudinalController Control longitudinal velocity. 
%   The longitudinal controller controls the velocity of a vehicle. Given
%   the reference velocity and the current velocity, the longitudinal
%   controller computes acceleration and deceleration commands.
%
%   lonController = HelperLongitudinalController creates a longitudinal  
%   controller system object, lonController, that computes acceleration  
%   and deceleration commands.  
%
%   lonController = HelperLongitudinalController(Name,Value) creates a
%   longitudinal controller object, with additional options specified
%   by one or more Name,Value pair arguments:
%
%   'Direction'           Driving direction of the vehicle: 1 for forward 
%                         motion and -1 for reverse motion. Note this   
%                         property is used only when running the system  
%                         object in MATLAB. In Simulink, direction is  
%                         provided as an input to the system object and
%                         this property is not used.
%
%                         Default: 1
%    
%   'Kp'                  Proportional gain of the controller.
%
%                         Default: 4
%
%   'Ki'                  Integral gain of the controller.
%
%                         Default: 1
%  
%   'MaxAcceleration'     Maximum longitudinal acceleration of the 
%                         vehicle. The output accelCmd is saturated
%                         to the range of [0, MaxAcceleration].
%
%                         Default: 3 (meters/second^2)
%
%   'MaxDeceleration'     Maximum longitudinal develeration of the 
%                         vehicle. The output decelCmd is saturated
%                         to the range of [0, MaxDeceleration].
%
%                         Default: 6 (meters/second^2)
%
%   'SampleTime'          Sample time of the controller. It specifies the
%                         integral time step of the controller.
%
%                         Default: 0.05 (seconds)
%
%   Step method syntax:
%   [accelCmd, decelCmd] = step(lonController, refVelocity, currVelocity)  
%   computes the acceleration command accelCmd and the deceleration command
%   decelCmd, based on the reference velocity refVelocity and the current 
%   velocity currVelocity. In both forward and reverse motion, accelCmd 
%   increases the longitudinal speed of the vehicle and is saturated to the 
%   range of [0, MaxAcceleration]. decelCmd decreases the longitudinal speed  
%   and is saturated to the range of [0, MaxDeceleration].
%   
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj) and y = obj() are equivalent. 
%
%   HelperLongitudinalController properties:
%   Direction            - Driving direction (only used in MATLAB)
%   Kp                   - Proportional gain                   
%   Ki                   - Integral gain 
%   MaxAcceleration      - Maximum longitudinal acceleration 
%   MaxDeceleration      - Maximum longitudinal deceleration 
%   SampleTime           - Sample time 
% 
%   HelperLongitudinalController methods:
%   step                 - Compute acceleration and deceleration commands
%   release              - Allow property value changes
%   reset                - Reset internal states to default
%   clone                - Create a copy of the object 
%   isLocked             - Locked status (logical)
%   
%   Notes
%   -----
%   - The HelperLongitudinalController computes outputs based on a discrete  
%     Proportional-Integral (PI) controller with integral anti-windup.
%
%   Example 1: Compute acceleration and deceleration commands 
%   ---------------------------------------------------------       
%   % Create a longitudinal controller object
%   lonController = HelperLongitudinalController;
%
%   % Set reference velocity and current velocity
%   refVelocity  = 2; % meters/second
%   currVelocity = 0; % meters/second
%       
%   % Compute control commands
%   [accelCmd, decelCmd] = lonController(refVelocity, currVelocity)
%
%   Example 2: Automated Parking Valet
%   ----------------------------------
%   % Steer a vehicle to the final spot in a parking lot. 
%   % <a href="matlab:web(fullfile(docroot, 'driving/examples/automated-parking-valet.html'))">View example</a>  
% 
%   See also lateralControllerStanley.

%   References
%   ----------
%   [1] Hoffmann, Gabriel M., Claire J. Tomlin, Michael Montemerlo, and 
%       Sebastian Thrun. "Autonomous automobile trajectory tracking for 
%       off-road driving: Controller design, experimental validation and 
%       racing." In American Control Conference, pp. 2296-2301, 2007.

%   Copyright 2018 The MathWorks, Inc.    

%#codegen
    properties
        %Kp Proportional gain, Kp
        %
        %   Default:      2.5
        Kp              = 2.5
        
        %Ki Integral gain, Ki
        %
        %   Default:      1
        Ki              = 1
        
        %Direction Driving direction 
        %   It has two possible values: 1 for forward motion and -1 for 
        %   reverse motion. This property is only used in MATLAB. 
        %   In Simulink, direction is provided as an input via the inport.
        %
        %   Default:      1
        Direction       = 1   
    end
    
    properties(DiscreteState, Hidden)
        ErrorIntegral  
    end
    
    properties(Nontunable)        
        %MaxAcceleration Maximum longitudinal acceleration (m/s^2)
        %
        %   Default:      3 (meters/second^2)
        MaxAcceleration = 3 
        
        %MaxDeceleration Maximum longitudinal deceleration (m/s^2)
        %
        %   Default:      6 (meters/second^2)
        MaxDeceleration = 6 
        
        %SampleTime Sample time (s)
        %
        %   Default:      0.05 (seconds)
        SampleTime      = 0.05 
    end
    
    properties(Nontunable, Logical)
        %HasResetInput Enable Reset input port
        %   Flag indicating if there is a reset input to reset
        %   the discrete state, ErrorIntegral. This property is used in 
        %   Simulink only. In MATLAB, use reset(obj) to reset the state.
        %
        %   Default:         false
        HasResetInput      = false
    end
    
    properties(Nontunable)
        %SampleTimeOption Sample time options
        %   The option allows selecting the source of sample time when using
        %   the system object in Simulink. The sample time can either be
        %   inherited from Simulink engine or explicitly specified by users.
        %
        %   Default:       'Specify'
        SampleTimeOption = 'Specify'
    end
    
    properties(Constant, Hidden)
        %SampleTimeOptionSet
        SampleTimeOptionSet = matlab.system.StringSet({'Specify', 'Inherit'})
    end
    
    properties(Access = private, Nontunable, Logical)
        %HasDirectionInput Flag indicating the source of direction. In
        %   MATLAB direction is set as a property while in Simulink it is
        %   provided as an input to the inport. It is set in setupImpl.
        %
        %   Default:         true
        HasDirectionInput  = true
    end
    
    %----------------------------------------------------------------------
    % Setter
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function set.Kp(obj, Kp)
            
            validateattributes(Kp, {'double', 'single'}, ...
                {'nonnan', 'real', 'scalar', 'nonempty', 'positive'}, ...
                mfilename, 'Kp');
            obj.Kp = Kp;
        end
        
        %------------------------------------------------------------------
        function set.Ki(obj, Ki)
            
            validateattributes(Ki, {'double', 'single'}, ...
                {'nonnan', 'real', 'scalar', 'nonempty', 'positive'}, ...
                mfilename, 'Ki');
            obj.Ki = Ki;
        end
        
        %------------------------------------------------------------------
        function set.MaxAcceleration(obj, maxAcc)
            
            validateattributes(maxAcc, {'double', 'single'}, ...
                {'nonnan', 'real', 'scalar', 'nonempty', 'positive'}, ...
                mfilename, 'MaxAcceleration');
            obj.MaxAcceleration = maxAcc;
        end
        
        %------------------------------------------------------------------
        function set.MaxDeceleration(obj, maxDec)
            
            validateattributes(maxDec, {'double', 'single'}, ...
                {'nonnan', 'real', 'scalar', 'nonempty', 'positive'}, ...
                mfilename, 'MaxDeceleration');
            obj.MaxDeceleration = maxDec;
        end
        
        %------------------------------------------------------------------
        function set.SampleTime(obj, Ts)
            
            validateattributes(Ts, {'double', 'single'}, ...
                {'nonnan', 'real', 'scalar', 'nonempty', 'positive'}, ...
                mfilename, 'SampleTime');
            obj.SampleTime = Ts;
        end
        
        %------------------------------------------------------------------
        function set.Direction(obj, direction)
            
            obj.checkDirection(direction, mfilename, 'Direction')
            obj.Direction = direction;
        end
    end
    
    %----------------------------------------------------------------------
    % Constructor
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function obj = HelperLongitudinalController(varargin)
            %HelperLongitudinalController Constructor
            setProperties(obj,nargin,varargin{:}, 'Kp', 'Ki', 'SampleTime', ...
                'Direction', 'MaxAcceleration', 'MaxDeceleration');
        end
    end
    
    %----------------------------------------------------------------------
    % Algorithm
    %----------------------------------------------------------------------
    methods (Access = protected)      
        %------------------------------------------------------------------
        function [accelCmd, decelCmd] = stepImpl(obj, refVel, currVel, varargin)
            
            if obj.HasDirectionInput
                narginchk(4,5);
                direction = varargin{1};
                if obj.HasResetInput
                    resetErrorIntegral = varargin{2};
                    if resetErrorIntegral
                        obj.resetImpl();
                    end
                end
            else
                narginchk(3,3);
                direction = obj.Direction;
            end
            
            % Validate Direction
            obj.checkDirection(direction, 'validateInputsImpl', 'Direction');
                        
            % Get sample time  
            sts = getSampleTime(obj);
            sampleTime = sts.SampleTime;
            
            % Compute control command based on the following formular
            %
            % u(k) = sat(Kp * e(k) + sat(Ki * integral(e(k)), lb, ub), lb, ub)
            %
            % where sat(x, lb, ub) = min(max(x, lb), ub)
            % lb is the lower limit for saturation
            % ub is the upper limit for saturation
            %
            % In forward motion, lb = -MaxDeceleration, ub = MaxAcceleration
            % in reverse motion, lb = -MaxAcceleration, ub = MaxDeceleration 
            %
            % see also Equation (10) and (11) in reference [1].
            eRef              = refVel - currVel;            
            obj.ErrorIntegral = obj.ErrorIntegral + eRef*sampleTime;
            ctrlCmd           = saturate(obj, obj.Kp*eRef + ...
                obj.Ki*saturate(obj, obj.ErrorIntegral, direction), direction);
            
            % Map the control command to acceleration and deceleration
            % commands based on the driving direction. One of the two
            % commands is always equal to zero        
            if ctrlCmd >= 0 % increase velocity
                if direction == 1
                    accelCmd = ctrlCmd;
                    decelCmd = 0;
                else % reversed driving
                    accelCmd = 0;
                    decelCmd = ctrlCmd;
                end
            else % ctrlCmd < 0, decrease velocity
                if direction == 1
                    accelCmd = 0;
                    decelCmd = -ctrlCmd; 
                else
                    accelCmd = -ctrlCmd;
                    decelCmd = 0;
                end
            end
        end
        
       %-------------------------------------------------------------------
        function ctrlCmd = saturate(obj, ctrlCmd, dir)
            % Saturate Saturate control command
            if dir == 1 % forward motion
                lb = -obj.MaxDeceleration;
                ub =  obj.MaxAcceleration;
            else % reversed motion
                lb = -obj.MaxAcceleration;
                ub =  obj.MaxDeceleration;
            end   
            ctrlCmd = min(max(ctrlCmd, lb), ub);
        end
    end
    
    %----------------------------------------------------------------------
    % Common functions
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj, ~, ~, varargin)
            if isSimulinkBlock(obj) % In Simulink
                obj.HasDirectionInput = true;
            else % In MATLAB
                obj.HasDirectionInput = false;
            end
        end
        
        %------------------------------------------------------------------
        function validateInputsImpl(obj, refVel, currVel, varargin)
            % validateInputsImpl Validate inputs before setupImpl is called
            
            obj.checkVelocity(refVel,  'validateInputsImpl', 'refVelocity');
            obj.checkVelocity(currVel, 'validateInputsImpl', 'currVelocity');
            
            if obj.HasResetInput && isSimulinkBlock(obj) % In Simulink
                % Validate Reset signal
                validateattributes(varargin{2}, {'double', 'single', 'logical'}, ...
                    {'nonnan', 'real', 'finite', 'scalar', 'nonempty'}, ...
                    'validateInputsImpl', 'Reset');
            end
        end

        function flag = isInputSizeMutableImpl(obj, ~)
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end
        
        %------------------------------------------------------------------
        function [out1, out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [1 1];
            out2 = [1 1];
        end
        
        %------------------------------------------------------------------
        function [out1, out2] = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            out1 = propagatedInputDataType(obj,1);
            out2 = propagatedInputDataType(obj,1);
        end
        
        %------------------------------------------------------------------
        function [out1, out2] = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
        end
        
        %------------------------------------------------------------------
        function [out1, out2] = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            out1  = true;
            out2 = true;
        end
        
        %------------------------------------------------------------------
        function num = getNumInputsImpl(obj)
            %getNumInputsImpl return number of inputs
            if obj.HasDirectionInput && isSimulinkBlock(obj) % In Simulink
                num = 3;
                if obj.HasResetInput
                    num = 4;
                end
            else
                num = 2;
            end
        end
        
        %------------------------------------------------------------------
        function num = getNumOutputsImpl(obj)
            %getNumOutputsImpl return number of outputs
            num = 2;
        end
        
        %------------------------------------------------------------------
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            obj.HasDirectionInput = s.HasDirectionInput; 

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
        
        %------------------------------------------------------------------
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            s.HasDirectionInput = obj.HasDirectionInput;
        end
        
        %------------------------------------------------------------------
        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = ["Helper", "Longitudinal", "Controller"];
        end
        
        %------------------------------------------------------------------
        function varargout = getInputNamesImpl(obj)
            % Return input port names for System block
            varargout{1} = 'RefVelocity';
            varargout{2} = 'CurrVelocity';
            if obj.HasDirectionInput && isSimulinkBlock(obj) % In Simulink
                varargout{3} = 'Direction';
                if obj.HasResetInput
                    varargout{4} = 'Reset';
                end
            end
        end
        
        %------------------------------------------------------------------
        function [name1,name2] = getOutputNamesImpl(obj)
            % Return output port names for System block
            name1 = 'AccelCmd';
            name2 = 'DecelCmd';
        end
        
        %------------------------------------------------------------------
        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,~)
            % Return size, data type, and complexity of discrete-state
            % specified in name
            sz = [1 1];
            dt = propagatedInputDataType(obj,1);
            cp = false;
        end
        
        %------------------------------------------------------------------
        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            if isSimulinkBlock(obj) && ...
                    strcmp(obj.SampleTimeOption, 'Inherit')
                sts = obj.createSampleTime("Type", "Inherited");
            else
                sts = obj.createSampleTime("Type", "Discrete", ...
                    "SampleTime", obj.SampleTime);
            end
        end
        
        %------------------------------------------------------------------
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.ErrorIntegral = 0;
        end
        
        %------------------------------------------------------------------
        function releaseImpl(obj)
            % Release resources, such as file handles
        end
        
        %------------------------------------------------------------------
        function flag = isInactivePropertyImpl(obj,prop)
            % Return false if property is visible based on object 
            % configuration, for the command line and System block dialog
            
            if strcmp(prop, 'Direction')
                flag = obj.HasDirectionInput && isSimulinkBlock(obj);
            elseif strcmp(prop, 'SampleTime')
                flag = isSimulinkBlock(obj) && strcmp(obj.SampleTimeOption, 'Inherit');
            elseif strcmp(prop, 'SampleTimeOption') || strcmp(prop, 'HasResetInput')
                flag = ~isSimulinkBlock(obj);
            else
                flag = false;
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Utility functions
    %---------------------------------------------------------------------- 
    methods (Access = protected)
        function flag = isSimulinkBlock(obj)
            flag = getExecPlatformIndex(obj); % 0 for MATLAB, 1 for Simulink
        end
    end
    
    %----------------------------------------------------------------------
    % Simulink dialog
    %---------------------------------------------------------------------- 
    methods(Access = protected, Static)        
        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            controllerGroup = matlab.system.display.Section(...
                'Title', 'Controller settings', ...
                'PropertyList', {'Kp', 'Ki', 'Direction', ...
                'SampleTimeOption', 'SampleTime', 'HasResetInput'});
            
            vehicleGroup = matlab.system.display.Section(...
                'Title', 'Vehicle parameters', ...
                'PropertyList', {'MaxAcceleration', 'MaxDeceleration'});
            
            group = [controllerGroup, vehicleGroup];
        end
    end
    
    %----------------------------------------------------------------------
    % Validate data
    %----------------------------------------------------------------------
    methods(Access = private)
        %------------------------------------------------------------------
        function checkVelocity(~, velocity, fcnName, argName)
            validateattributes(velocity, {'double', 'single'}, ...
                {'nonnan', 'real', 'finite', 'scalar', 'nonempty'}, ...
                fcnName, argName);
        end
        
        %------------------------------------------------------------------
        function checkDirection(~, direction, fcnName, argName)
            validateattributes(direction, {'double', 'single'}, ...
                {'nonnan', 'real', 'finite', 'scalar', 'nonempty'}, ...
                fcnName, argName);
            
            isValidDirection = any(direction == [1, -1]);
            
            coder.internal.errorIf(~isValidDirection, ...
                'driving:lateralControllerStanley:invalidDirection');
        end
    end
end
