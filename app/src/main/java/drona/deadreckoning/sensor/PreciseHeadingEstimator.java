package drona.deadreckoning.sensor;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;

public class PreciseHeadingEstimator {
    
    private static final float MAG_WEIGHT = 0.02f;
    private static final float ALPHA = 0.96f;
    
    private float[] gravityValues = null;
    private float[] magValues = null;
    private float[] gyroValues = null;
    
    private double gyroHeading = 0;
    private double magHeading = 0;
    private double filteredHeading = 0;
    
    private long lastTimestamp = 0;
    
    private KalmanFilter headingKalman;
    private float[] rotationMatrix = new float[9];
    private float[] orientationAngles = new float[3];
    
    public PreciseHeadingEstimator() {
        headingKalman = new KalmanFilter(0.001, 0.05, 0, 1);
    }
    
    public void updateGravity(float[] values) {
        this.gravityValues = values;
    }
    
    public void updateMagneticField(float[] values) {
        this.magValues = values;
    }
    
    public void updateGyroscope(float[] values, long timestamp) {
        if (lastTimestamp == 0) {
            lastTimestamp = timestamp;
            return;
        }
        
        float dt = (timestamp - lastTimestamp) / 1_000_000_000.0f;
        lastTimestamp = timestamp;
        
        gyroValues = values.clone();
        
        double gyroHeadingDelta = -gyroValues[2] * dt;
        gyroHeading += gyroHeadingDelta;
        
        while (gyroHeading > Math.PI) gyroHeading -= 2 * Math.PI;
        while (gyroHeading < -Math.PI) gyroHeading += 2 * Math.PI;
    }
    
    public void updateGyroscope(SensorEvent event) {
        updateGyroscope(event.values, event.timestamp);
    }
    
    public double getHeading() {
        if (gravityValues != null && magValues != null) {
            calculateMagnetometerHeading();
        }
        
        if (gyroValues != null) {
            filteredHeading = ALPHA * gyroHeading + (1 - ALPHA) * magHeading;
        } else {
            filteredHeading = magHeading;
        }
        
        double kalmanHeading = headingKalman.update(filteredHeading);
        
        return Math.toDegrees(kalmanHeading);
    }
    
    private void calculateMagnetometerHeading() {
        boolean success = SensorManager.getRotationMatrix(
            rotationMatrix, null, gravityValues, magValues
        );
        
        if (success) {
            SensorManager.getOrientation(rotationMatrix, orientationAngles);
            magHeading = orientationAngles[0];
        }
    }
    
    public double getHeadingDegrees() {
        return getHeading();
    }
    
    public void reset() {
        gyroHeading = 0;
        magHeading = 0;
        filteredHeading = 0;
        lastTimestamp = 0;
        headingKalman.reset(0, 1);
    }
    
    public float getAccuracy() {
        return gravityValues != null && magValues != null ? 1.0f : 0.0f;
    }
    
    public boolean hasValidData() {
        return gravityValues != null && magValues != null;
    }

    public float[] getGravity() {
        return gravityValues;
    }

    public float[] getMagneticField() {
        return magValues;
    }

    public float[] getGyroscope() {
        return gyroValues;
    }
}
