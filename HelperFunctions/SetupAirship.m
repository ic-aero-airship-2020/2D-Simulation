function [airship] = SetupAirship(airshipConfig,kinematicsConfig,sensorConfig)
    %airship definition
    airship.config.airship = airshipConfig;
    airship.config.sensors = sensorConfig;

    % driver initialisation
    wheelSpeedRange = [kinematicsConfig.minThrust, kinematicsConfig.maxThrust];
    airship.diffDrive = differentialDriveKinematics(...
        "VehicleInputs","VehicleSpeedHeadingRate",...
        "WheelSpeedRange",wheelSpeedRange,...
        "TrackWidth",kinematicsConfig.thrustVectorDist,...
        "VehicleInputs","VehicleSpeedHeadingRate");
    airship.controller = controllerPurePursuit(...
        'DesiredLinearVelocity',kinematicsConfig.cruiseSpeed,...
        'MaxAngularVelocity',kinematicsConfig.maxAngularVelocity);

    % sensor initialisation
    airship.sensors.s1 = rangeSensor;
    airship.sensors.s1.Range = [0,sensorConfig.MaxRange];
    airship.sensors.s1.HorizontalAngle = [-sensorConfig.FieldOfVision/2, sensorConfig.FieldOfVision/2];
    
    airship.sensors.s2 = rangeSensor;
    airship.sensors.s2.Range = [0,sensorConfig.MaxRange];
    airship.sensors.s2.HorizontalAngle = [-sensorConfig.FieldOfVision/2, sensorConfig.FieldOfVision/2];
    
    airship.sensors.s3 = rangeSensor;
    airship.sensors.s3.Range = [0,sensorConfig.MaxRange];
    airship.sensors.s3.HorizontalAngle = [-sensorConfig.FieldOfVision/2, sensorConfig.FieldOfVision/2];
    
    airship.sensors.s4 = rangeSensor;
    airship.sensors.s4.Range = [0,sensorConfig.MaxRange];
    airship.sensors.s4.HorizontalAngle = [-sensorConfig.FieldOfVision/2, sensorConfig.FieldOfVision/2];
    
    airship.sensors.s5 = rangeSensor;
    airship.sensors.s5.Range = [0,sensorConfig.MaxRange];
    airship.sensors.s5.HorizontalAngle = [-sensorConfig.FieldOfVision/2, sensorConfig.FieldOfVision/2];
    
    airship.sensors.s6 = rangeSensor;
    airship.sensors.s6.Range = [0,sensorConfig.MaxRange];
    airship.sensors.s6.HorizontalAngle = [-sensorConfig.FieldOfVision/2, sensorConfig.FieldOfVision/2];
    
end