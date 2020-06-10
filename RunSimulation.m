%% Cleaning environment
clear
clc
close all

%% Step 0. Setting up the simualation
% Adding folders to path
addpath('./HelperFunctions/');
addpath('./Map/');

% Choosing the map to run the simulation on
% ######## Choose map complexity ########
% 1 - simple map complexity
% 2 - complex map complexity
% 3 - imperial map complexity
complexity = 2;
withManualFlight = false;
withSLAM = false;
% #######################################

% Configuring airship and its kinematics
% ######## Defining key variables ########
airshipConfig.maxWidth = 1.5;
airshipConfig.maxHeight = 1.5;

kinematicsConfig.thrustVectorDist = 1.5;
kinematicsConfig.maxThrust = Inf;
kinematicsConfig.minThrust = -Inf;
kinematicsConfig.cruiseSpeed = 2; % m/s
kinematicsConfig.maxAngularVelocity = Inf; % rads/s

sensorConfig.MaxRange = 2; % m
sensorConfig.FieldOfVision = 27/180*pi; % rad

sensorConfig.Offset1 =  60   /180*pi; % rad
sensorConfig.Offset2 = -60   /180*pi; % rad
sensorConfig.Offset3 =  180  /180*pi; % rad
sensorConfig.Offset4 =  120  /180*pi; % rad
sensorConfig.Offset5 =  0    /180*pi; % rad
sensorConfig.Offset6 = -120  /180*pi; % rad
% #########################################

% Setting up environment
[refmap,manualPath] = SetupSimulationEnvironment(complexity);
airship = SetupAirship(airshipConfig,kinematicsConfig,sensorConfig);

%% Step 1. Setting up the graphics
% blank map to be populated
mapXDim = refmap.XWorldLimits(2);
mapYDim = refmap.YWorldLimits(2);
genmap = binaryOccupancyMap(mapXDim,mapYDim,10);

% visualising the maps
fig = figure('Name','Reference Map','WindowState', 'maximized');
refax = subplot(1,2,1);
show(refmap)
title('Ground truth map - Actual Layout');
genax = subplot(1,2,2);
show(genmap)
title('generated map - Map on the airship');

%% Step 2 Setting up simulation
% Setting up time intervals
sampleTime = 0.05;             % Sample time [s]
t = 0:sampleTime:1000;         % Time array

% Setting up poses
initPose = [manualPath(1,1), manualPath(1,2), pi];
poses = zeros(3,numel(t));    % Pose matrix
estposes = zeros(3,numel(t));    % Estimated Pose matrix
poses(:,1) = initPose';
goalPoint = [manualPath(end,1) manualPath(end,2)]';
goalPose = [manualPath(end,1) manualPath(end,2) 0];

% Set iteration rate
r = rateControl(1/sampleTime);

% Setting up airship
airshipRad = airship.config.airship.maxWidth/2;
[x,y] = CirclePoints(initPose(1),initPose(2),airshipRad,17);
xAxesLim = get(refax,'XLim');
lineLength = diff(xAxesLim)/40;
len = max(lineLength,2*airshipRad);
xp = [initPose(1), initPose(1)+(len*cos(initPose(3)))];
yp = [initPose(2), initPose(2)+(len*sin(initPose(3)))];

% Showing on plots
hold(refax,'on')
airshipHandle = plot(refax,x,y,'b','LineWidth',1.5);
orientaionHandle = plot(refax,xp,yp,'b','LineWidth',1.5);
flowPathnHandle = plot(refax,initPose(1),initPose(2),'g-');

startPointHandle = plot(refax,initPose(1),initPose(2),'rx','LineWidth',1.5);
endPointHandle = plot(refax,goalPose(1),goalPose(2),'mx','LineWidth',1.5);

if withManualFlight
    manualPathHandle = plot(refax,manualPath(:,1),manualPath(:,2),'r-');
    legend([manualPathHandle,flowPathnHandle,startPointHandle,endPointHandle],{'Manual Planned Path','Flown Path','Start Point','Goal Point'},'Location','southwest');
    airship.controller.Waypoints = manualPath;
    
else
    % Defining state space of the vehicle
    bounds = [[0 mapXDim]; [0 mapYDim]; [-pi pi]];
    
    ss = stateSpaceDubins(bounds);
    minTuriningRadius = airshipRad;
    ss.MinTurningRadius = minTuriningRadius;
    
    % Setting up validator
    stateValidator = validatorOccupancyMap(ss);
    validationDistance = 0.1;
    
    alternateStateValidator = validatorOccupancyMap();
    
    plannedPathHandle = plot(refax,initPose(1),initPose(2),'r-');
    legend([plannedPathHandle,flowPathnHandle,startPointHandle,endPointHandle],{'Autonomous Planned Path','Flown Path','Start Point','Goal Point'},'Location','southwest');
end


hold(refax,'off')

hold(genax,'on')
airshipPointHandle = plot(genax,goalPoint(1),goalPoint(2),'rx');
hold(genax,'off')

% Setting up slam algoritm
maxLidarRange = airship.config.sensors.MaxRange;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

slamAlg.LoopClosureThreshold = 360;  
slamAlg.LoopClosureSearchRadius = maxLidarRange;

figure('Name','SLAM Map');
slamAx = show(slamAlg);

firstLoopClosure = false;
lidarData = cell(1,numel(t));

for idx = 1:numel(t)
    reversePath = false;
    
    position = poses(:,idx)';
    currPose = position(1:2);
    angleOffset1 = airship.config.sensors.Offset1;
    angleOffset2 = airship.config.sensors.Offset2;
    angleOffset3 = airship.config.sensors.Offset3;
    angleOffset4 = airship.config.sensors.Offset4;
    angleOffset5 = airship.config.sensors.Offset5;
    angleOffset6 = airship.config.sensors.Offset6;
    
    position1 = position;
    position2 = position;
    position3 = position;
    position4 = position;
    position5 = position;
    position6 = position;
    
    position1(3) = wrapToPi(position1(3)+angleOffset1);
    position2(3) = wrapToPi(position2(3)+angleOffset2);
    position3(3) = wrapToPi(position3(3)+angleOffset3);
    position4(3) = wrapToPi(position4(3)+angleOffset4);
    position5(3) = wrapToPi(position5(3)+angleOffset5);
    position6(3) = wrapToPi(position6(3)+angleOffset6);

    % End if pathfollowing is vehicle has reached goal position within tolerance of 0.2m
    dist = norm(goalPoint'-currPose);
    if (dist < 1)
        disp("Goal position reached")
        break;
    end

    % Update map by taking sensor measurements
    [ranges1, angles1] = airship.sensors.s1(position1, refmap);
    [ranges2, angles2] = airship.sensors.s2(position2, refmap);
    [ranges3, angles3] = airship.sensors.s3(position3, refmap);
    [ranges4, angles4] = airship.sensors.s4(position4, refmap);
    [ranges5, angles5] = airship.sensors.s5(position5, refmap);
    [ranges6, angles6] = airship.sensors.s6(position6, refmap);
    
    ranges = [ranges1;ranges2;ranges3;ranges4;ranges5;ranges6];
    angles = [angles1+angleOffset1;angles2+angleOffset2;angles3+angleOffset3;angles4+angleOffset4;angles5+angleOffset5;angles6+angleOffset6];
    
    scan = lidarScan(ranges,angles);
    lidarData{idx} = scan;
    validScan = removeInvalidData(scan,'RangeLimits',[0,airship.config.sensors.MaxRange]);

    % Update slam every 20 scans
    if withSLAM
        if mod(idx,20)==0
            [isScanAccepted,loopClosureInfo,optimizationInfo] = addScan(slamAlg, validScan);
            show(slamAlg, 'Parent', slamAx);

            if optimizationInfo.IsPerformed && ~firstLoopClosure
                firstLoopClosure = true;
                show(slamAlg,'Poses','off','Parent',slamAx);
                hold(slamAx, 'on');
                show(slamAlg.PoseGraph,'Parent', slamAx); 
                hold(slamAx, 'off');
                title('First loop closure');
                snapnow
            end
        end
    end
    
    insertRay(genmap,position,validScan,airship.config.sensors.MaxRange);
    show(genmap,'Parent',genax,'FastUpdate',1);
    
    if ~withManualFlight
        % map is inflated to account for the width of the airship
        mapInflated = copy(genmap);
        inflate(mapInflated, airshipRad);
        stateValidator.Map = mapInflated;
        
        % Defining validation distance
        stateValidator.ValidationDistance = validationDistance; % m 
        
        planner = plannerRRTStar(ss, stateValidator);
        planner.MaxConnectionDistance = 2.0;
        planner.MaxIterations = 30000;

        planner.GoalReachedFcn = @CheckIfGoalReached;
        [pthObj, solnInfo] = plan(planner, position, goalPose);
        
        if ~solnInfo.IsPathFound
%             alternateStateValidator.Map = mapInflated;
%             
%             planner = plannerHybridAStar(alternateStateValidator,'MinTurningRadius',);
%             pthObj = plan(planner, position, goalPose);
            
            reversePath = true;
            disp('Used alternate planner');
        else
            waypointX = pthObj.States(:,1);
            waypointY = pthObj.States(:,2);
        end
        
        
        set(plannedPathHandle,'xdata',waypointX,'ydata',waypointY);
        airship.controller.Waypoints = [waypointX,waypointY];
    end
    
    
    
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = airship.controller(poses(:,idx));

    % Perform forward discrete integration step
    vel = derivative(airship.diffDrive, poses(:,idx), [vRef wRef]);
    if reversePath
        vel = [0; 0; -15.7];
    end
    poses(:,idx+1) = poses(:,idx) + vel*sampleTime; 

    % Update position
    [xc,yc] = CirclePoints(poses(1, idx+1),poses(2, idx+1),airshipRad,17);
    set(airshipHandle,'xdata',xc,'ydata',yc);
    set(airshipPointHandle,'xdata',poses(1, idx+1),'ydata',poses(2, idx+1));
    xp = [poses(1, idx+1), poses(1, idx+1)+(len*cos(poses(3, idx+1)))];
    yp = [poses(2, idx+1), poses(2, idx+1)+(len*sin(poses(3, idx+1)))];
    set(orientaionHandle,'xdata',xp,'ydata',yp);
    
    % Update path flown
    set(flowPathnHandle,'xdata',poses(1, 1:idx),'ydata',poses(2, 1:idx));
    
    % waiting to iterate at the proper rate
    waitfor(r);
end