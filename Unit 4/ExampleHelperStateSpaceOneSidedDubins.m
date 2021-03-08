classdef ExampleHelperStateSpaceOneSidedDubins < stateSpaceDubins
    %EXAMPLEHELPERSTATESPACEONESIDEDDUBINS State space for one-sided Dubins vehicles
    %   This state space is similar to stateSpaceDubins, but the vehicle
    %   can only either make left turns or make right turns. 
    
    properties
        %GoLeft
        GoLeft
    end
    
    properties
        %GoLeftInternal
        GoLeftInternal
        
        %DisabledTypes
        DisabledTypes
    end
    
    methods
        function obj = ExampleHelperStateSpaceOneSidedDubins(bounds, goLeft)
            %EXAMPLEHELPERSTATESPACEONESIDEDDUBINS Constructor
            obj@stateSpaceDubins(bounds);
            obj.Name = 'One-sided Dubins';
            obj.GoLeft = goLeft;
        end
        
        function dist = distance(obj, state1, state2)
            %DISTANCE Distance between two states
            %   Note that state1 is a M1-by-3 matrix, and state2 is a 
            %   M2-by-3 matric.
            %   M1 and M2 must be positive integers, if they are not equal
            %   to each other, then at least one of them must be 1.
            
            distRow = matlabshared.planning.internal.DubinsBuiltins.autonomousDubinsSegments(...
                double(state1), double(state2), obj.MinTurningRadius, 'optimal', obj.DisabledTypes);
            dist = distRow.';
            
        end
        
        function interpState = interpolate(obj, state1, state2, ratios)
            %INTERPOLATE Interpolate between two states
            [pathLength,segmentLengths,segmentTypes] = matlabshared.planning.internal.DubinsBuiltins.autonomousDubinsSegments(...
                state1, state2, obj.MinTurningRadius, 'optimal', obj.DisabledTypes);

            interpState = matlabshared.planning.internal.DubinsBuiltins.autonomousDubinsInterpolateSegments(...
                state1, state2, double(ratios) * pathLength(1), obj.MinTurningRadius, ...
                segmentLengths(:)', uint32(segmentTypes(:)'));

            interpState(:,3) = robotics.internal.wrapToPi(interpState(:,3));
        end
        
        function set.GoLeft(obj, goLeft)
            %set.GoLeft
            obj.setSide(goLeft);
        end
        
        function goLeft = get.GoLeft(obj)
            %get.GoLeft
            goLeft = obj.GoLeftInternal;
        end

    end
    
    methods (Access = protected)
        function propgrp = getPropertyGroups(obj)
            %getPropertyGroups Custom property group display

            propGroupDubins = getPropertyGroups@stateSpaceDubins(obj);
            
            propgrp = [propGroupDubins, matlab.mixin.util.PropertyGroup('GoLeft', 'One-Sided Dubins Properties:')];

        end
        
        function setSide(obj, goLeft)
            %setSide
            obj.GoLeftInternal = goLeft;
            if goLeft % left only
                obj.DisabledTypes = {'LSR', 'RSL', 'RSR', 'RLR', 'LRL'};
            else      % right only
                obj.DisabledTypes = {'LSR', 'RSL', 'LSL', 'RLR', 'LRL'};
            end
        end
    end
end

