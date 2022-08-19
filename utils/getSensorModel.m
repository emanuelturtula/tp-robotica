function [sensorModel] = getSensorModel(map, limits, num_scans)
    sensorModel = likelihoodFieldSensorModel;
    sensorModel.Map = map;
    sensorModel.SensorLimits = limits;
    sensorModel.MaxLikelihoodDistance = 5;
    sensorModel.NumBeams = 500;
    sensorModel.ExpectedMeasurementWeight = 0.8;
    sensorModel.MeasurementNoise = 0.1;
    sensorModel.RandomMeasurementWeight = 0.1;
end

