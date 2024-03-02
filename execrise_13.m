
%% Kalman Filter vs Zero-degree Least Squares 

%% Generate noisy measurements
numMeasurements = 100;
truePosition = 10;
measurementNoiseStd = 0.5;
measurements = truePosition + measurementNoiseStd * randn(numMeasurements, 1);

%% Kalman filter with zero degree
initialPositionEstimate = measurements(1);
initialEstimationErrorCovariance = 1;
positionEstimatesKF = zeros(numMeasurements, 1);
positionEstimatesKF(1) = initialPositionEstimate;
estimationErrorCovariance = initialEstimationErrorCovariance;

for i = 2:numMeasurements
    % Prediction step
    predictedPositionEstimate = positionEstimatesKF(i-1);
    predictedEstimationErrorCovariance = estimationErrorCovariance;
    
    % Update step
    measurementResidual = measurements(i) - predictedPositionEstimate;
    measurementNoise = measurementNoiseStd^2;
    kalmanGain = predictedEstimationErrorCovariance / (predictedEstimationErrorCovariance + measurementNoise);
    positionEstimatesKF(i) = predictedPositionEstimate + kalmanGain * measurementResidual;
    estimationErrorCovariance = (1 - kalmanGain) * predictedEstimationErrorCovariance;
end

%% Least squares with zero degree
positionEstimateLS = mean(measurements);

%% Plot results
time = 1:numMeasurements;
figure;
hold on;
plot(time, measurements, 'bo', 'DisplayName', 'Measurements');
plot(time, positionEstimatesKF, 'r-', 'DisplayName', 'Kalman Filter');
plot(time, positionEstimateLS * ones(numMeasurements, 1), 'g-', 'DisplayName', 'Least Squares');
plot(time, truePosition * ones(numMeasurements, 1), 'k--', 'DisplayName', 'True Position');
xlabel('Time');
ylabel('Position');
legend('Location', 'best');
grid on;
hold off;


% Least squares with regression and interest rate
estimatedPositionLS = mean(measurements);
estimatedRegressionCoeffLS = sum((measurements - truePosition) ./ (1:numMeasurements)') / numMeasurements;
estimatedInterestRateLS = sum((measurements - truePosition) ./ (1:numMeasurements)') / numMeasurements;

% Error variances for Kalman filter
positionErrorVarianceKF = estimationErrorCovariance;

% MSE for least squares
positionMSELS = mean((truePosition - estimatedPositionLS).^2);
interestRateMSELS = mean((measurements - estimatedInterestRateLS).^2);

disp("Kalman Filter Error Variance:");
disp("Position: " + positionErrorVarianceKF);

disp("Least Squares MSE:");
disp("Position: " + positionMSELS);
disp("Interest Rate: " + interestRateMSELS);





