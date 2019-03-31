classdef HelperFixedRate < handle
    %HelperFixedRate Execute a loop at a fixed frequency
    %   The HelperFixedRate class allows you to run a loop at a fixed
    %   frequency.
    %
    %   NOTE: The name of this class and it's functionality may change
    %   without notice in a future release, or the class itself may be
    %   removed.
    %
    %   fr = HelperFixedRate(desiredRate) creates a HelperFixedRate object
    %   fr that executed a loop at a fixed frequency equal to desiredRate.
    %   desiredRate is specified in Hz (number of executions per second).
    %
    %   HelperFixedRate properties:
    %   DesiredRate         - Desired execution rate (Hz)
    %   DesiredPeriod       - Desired time period between executions (seconds)
    %   TotalElapsedTime    - Elapsed time since construction or reset (seconds)
    %
    %   HelperFixedRate methods:
    %   waitfor - Pause execution to achieve desired rate
    %   reset   - Reset the HelperFixedRate object
    %
    %   Notes
    %   -----
    %   - The accuracy of rate execution is dependent on  scheduling
    %     resolution of the operating system and the level of other system
    %     activity. 
    %
    %   - This object relies on the PAUSE function. If the PAUSE is
    %     disabled, rate execution will not be accurate.
    %
    %
    %   Example
    %   -------
    %   % Create a HelperFixedRate object to execute at 5 Hz
    %   fr = HelperFixedRate(5);
    %
    %   % Start the loop
    %   tic
    %   for n = 1 : 10
    %       toc
    %       x = rand(100);
    %       waitfor(fr)
    %   end
    %
    %
    %   See also robotics.Rate (requires Robotics System Toolbox(TM)).
    
    properties (SetAccess = protected)
       %DesiredRate - Desired Execution rate (Hz)
        %   The execution rate (Hz) of a looping code segment that the
        %   object maintains. This is the inverse of the DesiredPeriod.
        DesiredRate
        
        %DesiredPeriod - Desired time period between executions (seconds).
        %   This is the inverse of the DesiredRate.
        DesiredPeriod
    end
    
    properties (Dependent, SetAccess = protected)
        %TotalElapsedTime - Elapsed time since object construction or reset (seconds)
        %   Accumulated time (in seconds) that has elapsed since the rate
        %   object was constructed or reset.
        %
        %   Default: 0
        TotalElapsedTime
    end
    
    properties (Access = protected)
        %StartTime - Time when the clock starts
        StartTime
        
        %ExecutionStartTime - Track the reference time for future executions
        %    All loop executions should occur at integer multiples of
        %    DesiredPeriod, relative to the baseline ExecutionStartTime. 
        %
        %    executionTime = ExecutionStartTime + i * DesiredPeriod, where
        %    i is a non-negative integer.
        ExecutionStartTime
        
        %NextExecutionIndex - Track the expected index for the next execution
        %   This index refers to the integer multiple of DesiredPeriod, at
        %   which the next execution should occur. The next execution time
        %   can be calculated as follows:
        %   nextExecutionTime = ExecutionStartTime + NextExecutionIndex*DesiredPeriod
        NextExecutionIndex
        
        %LastWakeTime - The completion time of the last WAITFOR call
        %   This is used to detect time retrogression.
        %   Default: NaN
        LastWakeTime
        
        %NumOverruns - Number of overruns since last construction or RESET
        %   This is used for calculating execution statistics.
        NumOverruns
        
        %PeriodCount - Number of completed waitfor periods since last construction or RESET
        %   A waitfor period is completed when the WAITFOR method runs to
        %   completion and no time retrogression occurs.
        %   This period count is used for calculating execution statistics.
        PeriodCount
    end
    
    methods
        %------------------------------------------------------------------
        function this = HelperFixedRate(desiredRate)
            
            validateattributes(desiredRate, {'numeric'}, ...
                {'scalar', 'real', 'positive', 'nonnan', 'finite'}, ...
                mfilename, 'desiredRate');
            
            this.DesiredRate     = double(desiredRate);
            this.DesiredPeriod   = 1/this.DesiredRate;
            
            reset(this);
        end
        
        %------------------------------------------------------------------
        function numMisses = waitfor(this)
            %waitfor - Pause execution to achieve desired rate
            %   waitfor(fr) pauses execution for an appropriate time to
            %   achieve the desired execution rate.
            %
            %   numMissed = waitfor(fr) returns the number of missed task
            %   executions since the last waitfor call. If numMisses>0, an
            %   overrun occurred.
            
            currentTime =  this.getElapsedTime();
            
            numMisses = 0;
            
            if currentTime < this.LastWakeTime
                % If time goes backward, reset time related parameters.
                this.recoverFromClockReset(currentTime);
                return;
            end
            
            % Estimate how long to sleep
            this.NextExecutionIndex = this.NextExecutionIndex + 1;
            sleepTime = this.NextExecutionIndex*this.DesiredPeriod + this.ExecutionStartTime - currentTime;
            
            % Handle overrun. If sleepTime is negative, an overrun
            % occurred.
            if sleepTime < 0
                numMisses = ceil(abs(sleepTime / this.DesiredPeriod));
                this.NumOverruns = this.NumOverruns + 1;
                
                % Slip
                this.NextExecutionIndex = 0;
                this.ExecutionStartTime = currentTime;
                sleepTime = 0;
            end
            
            % Sleep
            pause(sleepTime);
            
            currentTime = this.getElapsedTime();
            if currentTime > this.LastWakeTime
                this.PeriodCount    = this.PeriodCount + 1;
                this.LastWakeTime   = currentTime;
            else
                this.recoverFromClockReset(currentTime);
            end
        end
        
        %------------------------------------------------------------------
        function reset(this)
            
            this.NumOverruns        = 0;
            this.PeriodCount        = 0;
            this.NextExecutionIndex = 0;
            this.ExecutionStartTime = 0;
            this.LastWakeTime       = NaN;
            
            this.startTimeProvider();
        end
    end
    
    methods (Access = protected)
        %------------------------------------------------------------------
        function elapsedTime = getElapsedTime(this)
            
            elapsedTime = toc(this.StartTime);
        end
        
        %------------------------------------------------------------------
        function recoverFromClockReset(this, currentTime)
            
            this.NextExecutionIndex = 0;
            this.ExecutionStartTime = 0;
            this.startTimeProvider();
            
            this.LastWakeTime = currentTime;
            this.PeriodCount  = this.PeriodCount + 1;
        end
    end
    
    methods (Access = protected)
        %------------------------------------------------------------------
        function startTimeProvider(this)
            
            this.StartTime      = tic;
            this.LastWakeTime   = 0;
        end
    end

end