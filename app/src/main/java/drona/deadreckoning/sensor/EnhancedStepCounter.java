package drona.deadreckoning.sensor;

import java.util.ArrayList;
import java.util.List;

/**
 * Legacy enhanced pedestrian step counter from original 2016 pedometer base.
 *
 * @deprecated Retained for backwards compatibility with legacy activities.
 * Modern MARK-V vehicle navigation relies on [drona.deadreckoning.ml] neural motion engines
 * and [drona.deadreckoning.fusion.VehicleHierarchicalHybridEstimator] for vehicle dead reckoning.
 */
@Deprecated
public class EnhancedStepCounter {
    
    private static final int WINDOW_SIZE = 10;
    private static final double STEP_THRESHOLD_LOW = 9.5;
    private static final double STEP_THRESHOLD_HIGH = 11.5;
    private static final double STEP_MIN_TIME_MS = 250;
    
    private List<Double> accelerationWindow = new ArrayList<>();
    private List<Long> timestampWindow = new ArrayList<>();
    
    private int stepCount = 0;
    private long lastStepTime = 0;
    private boolean isPeakFound = false;
    private double peakValue = 0;
    private double currentStrideLength = 0.75;
    
    private KalmanFilter kalmanFilter;
    private double lastAcceleration = 0;
    
    public EnhancedStepCounter() {
        kalmanFilter = new KalmanFilter(0.01, 0.1, 0, 1);
    }
    
    public boolean detectStep(float[] linearAcceleration, long timestamp) {
        double magnitude = Math.sqrt(
            linearAcceleration[0] * linearAcceleration[0] +
            linearAcceleration[1] * linearAcceleration[1] +
            linearAcceleration[2] * linearAcceleration[2]
        );
        
        double filteredAcc = kalmanFilter.update(magnitude);
        
        accelerationWindow.add(filteredAcc);
        timestampWindow.add(timestamp);
        
        if (accelerationWindow.size() > WINDOW_SIZE) {
            accelerationWindow.remove(0);
            timestampWindow.remove(0);
        }
        
        return analyzeForStep(filteredAcc, timestamp);
    }
    
    private boolean analyzeForStep(double acceleration, long timestamp) {
        if (!isPeakFound) {
            if (acceleration > STEP_THRESHOLD_HIGH) {
                isPeakFound = true;
                peakValue = acceleration;
                return false;
            }
        } else {
            if (acceleration < STEP_THRESHOLD_LOW) {
                if (timestamp - lastStepTime > STEP_MIN_TIME_MS) {
                    lastStepTime = timestamp;
                    stepCount++;
                    isPeakFound = false;
                    return true;
                }
            } else if (acceleration > peakValue) {
                peakValue = acceleration;
            }
        }
        
        if (acceleration < STEP_THRESHOLD_LOW) {
            isPeakFound = false;
        }
        
        return false;
    }
    
    public void setStrideLength(double strideLengthMeters) {
        this.currentStrideLength = strideLengthMeters;
    }
    
    public double getStrideLength() {
        return currentStrideLength;
    }
    
    public int getStepCount() {
        return stepCount;
    }
    
    public void reset() {
        stepCount = 0;
        lastStepTime = 0;
        isPeakFound = false;
        accelerationWindow.clear();
        timestampWindow.clear();
        kalmanFilter.reset(0, 1);
    }
    
    public double getDistanceTraveled() {
        return stepCount * currentStrideLength;
    }
    
    public double getCurrentAcceleration() {
        if (accelerationWindow.isEmpty()) return 0;
        return accelerationWindow.get(accelerationWindow.size() - 1);
    }
    
    public double getFilteredAcceleration() {
        return kalmanFilter.getEstimate();
    }
    
    public List<Double> getAccelerationHistory() {
        return new ArrayList<>(accelerationWindow);
    }
}
