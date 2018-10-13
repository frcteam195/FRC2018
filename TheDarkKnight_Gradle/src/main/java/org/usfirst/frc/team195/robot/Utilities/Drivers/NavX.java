package org.usfirst.frc.team195.robot.Utilities.Drivers;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Rotation2d;

/**
 * Driver for a NavX board. Basically a wrapper for the {@link AHRS} class
 */
public class NavX {
    protected class Callback implements ITimestampedDataSubscriber {
        @Override
        public void timestampedDataReceived(long system_timestamp, long sensor_timestamp, AHRSUpdateBase update,
                Object context) {
            synchronized (NavX.this) {
                // This handles the fact that the sensor is inverted from our coordinate conventions.
                if (mLastSensorTimestampMs != kInvalidTimestamp && mLastSensorTimestampMs < sensor_timestamp) {
                    mYawRateDegreesPerSecond = 1000.0 * (-mYawDegrees - update.yaw)
                            / (double) (sensor_timestamp - mLastSensorTimestampMs);
                }
                mLastSensorTimestampMs = sensor_timestamp;
                mYawDegrees = -update.yaw;
            }
        }
    }

    protected AHRS mAHRS;

    protected Rotation2d mAngleAdjustment = Rotation2d.identity();
    protected double mYawDegrees;
    protected double mYawRateDegreesPerSecond;
    protected final long kInvalidTimestamp = -1;
    protected long mLastSensorTimestampMs;

    protected double mPrevAccelX = 0;
    protected double mPrevAccelY = 0;
    protected double mPrevTimeAccel = 0;

    private double mJerkCollisionThreshold = Constants.kCollisionDetectionJerkThreshold;
    private double mTippingThreshold = Constants.kTippingThresholdDeg;

    public NavX(SPI.Port spi_port_id) {
        mAHRS = new AHRS(spi_port_id, (byte) 200);
        resetState();
        mAHRS.registerCallback(new Callback(), null);
    }

    public boolean isPresent() {
        return mAHRS.isConnected();
    }

    public synchronized void reset() {
        mAHRS.reset();
        resetState();
    }

    public synchronized void zeroYaw() {
        mAHRS.zeroYaw();
        resetState();
    }

    private void resetState() {
        mLastSensorTimestampMs = kInvalidTimestamp;
        mYawDegrees = 0.0;
        mYawRateDegreesPerSecond = 0.0;
    }

    public synchronized void setAngleAdjustment(Rotation2d adjustment) {
        mAngleAdjustment = adjustment;
    }

    public synchronized double getRawYawDegrees() {
        return mYawDegrees;
    }

    public Rotation2d getYaw() {
        return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(getRawYawDegrees()));
    }

    public double getRoll() {
        return mAHRS.getRoll();
    }

    public double getPitch() {
        return mAHRS.getPitch();
    }

    public double getYawRateDegreesPerSec() {
        return mYawRateDegreesPerSecond;
    }

    public double getYawRateRadiansPerSec() {
        return 180.0 / Math.PI * getYawRateDegreesPerSec();
    }

    public double getRawAccelX() {
        return mAHRS.getRawAccelX();
    }

    public double getRawAccelY() {
        return mAHRS.getRawAccelY();
    }

    public double getRawAccelZ() {
        return mAHRS.getRawAccelZ();
    }

    public synchronized void setCollisionJerkThreshold(double jerkCollisionThreshold) {
        mJerkCollisionThreshold = jerkCollisionThreshold;
    }

    public synchronized void setTippingThreshold(double tippingThreshold) {
        mTippingThreshold = tippingThreshold;
    }

    public boolean isCollisionOccurring() {
        boolean collisionOccurring = false;

        double accelX = mAHRS.getWorldLinearAccelX();
        double accelY = mAHRS.getWorldLinearAccelY();


        double currTime = Timer.getFPGATimestamp();
        double dt = currTime-mPrevTimeAccel;

        double jerkX = (accelX - mPrevAccelX)/(dt);
        double jerkY = (accelY - mPrevAccelY)/(dt);

        if (Math.abs(jerkX) > mJerkCollisionThreshold || Math.abs(jerkY) > mJerkCollisionThreshold)
            collisionOccurring = true;

        mPrevAccelX = accelX;
        mPrevAccelY = accelY;

        if (mPrevTimeAccel == 0) {
            mPrevTimeAccel = currTime;
            return false;
        }

        mPrevTimeAccel = currTime;
        return collisionOccurring;
    }

    public boolean isTipping() {
        if (Math.abs(mAHRS.getPitch()) > mTippingThreshold || Math.abs(mAHRS.getRoll()) > mTippingThreshold)
            return true;

        return false;
    }
}
