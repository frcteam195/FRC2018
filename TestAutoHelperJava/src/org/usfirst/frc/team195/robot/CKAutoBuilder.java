package org.usfirst.frc.team195.robot;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;


/**
 * @author roberthilton
 * A class to simplify autonomous across many different motor controllers and robot configurations.
 * Ensure that all left drive motor controllers have the output direction moving in the same way
 * and that all right drive motor controllers have the output direction moving in the same way. 
 * This software is provided as-is with no warranty or formal support. Use at your own risk.
 * @param <T> The type of your motor controller. Can be any type of motor controller, PWM or CAN.
 */
public class CKAutoBuilder<T> extends Thread {	
	
	/**
	 * Constructor to build a new CKAutoBuilder. This should always be the first step of using CKAutoBuilder.
	 * Use this method if your motor controllers are setup as master/slaves.
	 * Usage: CKAutoBuilder<Spark> ckAuto = new CKAutoBuilder<Spark>(mySpark1, mySpark2, this);
	 * @param leftDrive The left drive motor controller.
	 * @param rightDrive The right drive motor controller.
	 * @param robot Use the "this" keyword in Robot.java to pass the robot into this parameter.
	 * @throws Exception
	 */
	public CKAutoBuilder(T leftDrive, T rightDrive, Robot robot) throws Exception {
		this.leftDrive = new ArrayList<T>();
		this.rightDrive = new ArrayList<T>();
		this.leftDrive.add(leftDrive);
		this.rightDrive.add(rightDrive);
		this.robot = robot;
		this.autoSteps = new ArrayList<CKAutoStep>();
		this.cheesyDriveHelper = new CheesyDriveHelper();
		this.useCheesyDrive = true;
	}
	
	/**
	 * Constructor to build a new CKAutoBuilder. This should always be the first step of using CKAutoBuilder.
	 * Usage: CKAutoBuilder<Spark> ckAuto = new CKAutoBuilder<Spark>(mySpark1, mySpark2, mySpark3, mySpark4, this);
	 * @param leftDrive1 The first left drive motor controller.
	 * @param leftDrive2 The second left drive motor controller.
	 * @param rightDrive1 The first right drive motor controller.
	 * @param rightDrive2 The second right drive motor controller.
	 * @param robot Use the "this" keyword in Robot.java to pass the robot into this parameter.
	 * @throws Exception
	 */
	public CKAutoBuilder(T leftDrive1, T leftDrive2, T rightDrive1, T rightDrive2, Robot robot) throws Exception {
		this(leftDrive1, rightDrive1, robot);
		this.leftDrive.add(leftDrive2);
		this.rightDrive.add(rightDrive2);
	}
	
	/**
	 * Constructor to build a new CKAutoBuilder. This should always be the first step of using CKAutoBuilder.
	 * Usage: CKAutoBuilder<Spark> ckAuto = new CKAutoBuilder<Spark>(mySpark1, mySpark2, mySpark3, mySpark4, mySpark5, mySpark6, this);
	 * @param leftDrive1 The first left drive motor controller.
	 * @param leftDrive2 The second left drive motor controller.
	 * @param leftDrive3 The third left drive motor controller.
	 * @param rightDrive1 The first right drive motor controller.
	 * @param rightDrive2 The second right drive motor controller.
	 * @param rightDrive3 The third right drive motor controller.
	 * @param robot Use the "this" keyword in Robot.java to pass the robot into this parameter.
	 * @throws Exception
	 */
	public CKAutoBuilder(T leftDrive1, T leftDrive2, T leftDrive3, T rightDrive1, T rightDrive2, T rightDrive3, Robot robot) throws Exception {
		this(leftDrive1, leftDrive2, rightDrive1, rightDrive2, robot);
		this.leftDrive.add(leftDrive3);
		this.rightDrive.add(rightDrive3);
	}
	
	/**
	 * Enable or disable Cheesy Drive mode.
	 * This mode changes the drive values to throttle and steering (curvature control), rather than direct output to the left and right motors.
	 * For direct output to left and right motors, disable Cheesy Drive mode.
	 * @param enable If true, Cheesy Drive is enabled. If false, Cheesy Drive is disabled.
	 */
	public void setCheesyDriveOnOff(boolean enable) {
		if (!this.isAlive())
			this.useCheesyDrive = enable;
	}
	
	/**
	 * Do not call this method. To start the thread, call start()
	 */
	@Override
    public void run() {		
		try {
			for (CKAutoStep ckAutoStep : autoSteps) {
				timeoutStart = Timer.getFPGATimestamp();
				while (timeoutElapsedTimeMS < ckAutoStep.getHoldTimeMS() && robot.isEnabled() && robot.isAutonomous()) {
					if (useCheesyDrive)
						setDriveOutput(cheesyDriveHelper.cheesyDrive(ckAutoStep.getThrottle(), ckAutoStep.getHeading(), false, false));
					else
						setDriveOutput(ckAutoStep.getThrottle(), ckAutoStep.getHeading());
					Thread.sleep(MIN_DRIVE_LOOP_TIME);
					timeoutEnd = Timer.getFPGATimestamp();
					timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
				}
				
				if (!(robot.isEnabled() && robot.isAutonomous())) {
					return;
				}
			}
		} catch (Exception e) {
			
		} 
		return;
    }
	
	/**
	 * Add a new step to the autonomous path for the robot.
	 * Usage Delay: ckAuto.addAutoStep(0, 0, 2000); //This will add a 2 second delay in the sequence of actions, no matter what drive mode is active.
	 * Usage Drive Forward Cheesy: ckAuto.addAutoStep(1, 0, 2000); //This will drive the robot forward for 2 seconds while in Cheesy Drive mode.
	 * Usage Drive Forward Direct: ckAuto.addAutoStep(1, 1, 2000); //This will drive the robot forward for 2 seconds in direct drive mode (first and second parameters represent left and right direct drive output).
	 * @param throttle The throttle value (forward and backward) for cheesy drive. If cheesy drive is disabled, this is the left side drive output. This value is always between -1 and 1.
	 * @param steer The heading value for cheesy drive (controls turning). If cheesy drive is disabled, this is the right side drive output. This value is always between -1 and 1.
	 * @param holdTimeMS The time for which to hold this drive action. The drive will run as specified for this value. Value is in milliseconds.
	 */
	public void addAutoStep(double throttle, double steer, int holdTimeMS) {
		if (!this.isAlive())
			autoSteps.add(new CKAutoStep(throttle, steer, holdTimeMS));
	}
	
	
	/**
	 * Simplest base method to drive the robot forward. Should not be used in conjunction with step mode.
	 */
	public synchronized void driveForward() {
		if (!this.isAlive()) {
			double driveSpeed = 0.5;
			double leftOutput = leftReversed ? -driveSpeed : driveSpeed;
			double rightOutput = rightReversed ? -driveSpeed : driveSpeed;
			setDriveOutput(leftOutput, rightOutput);
		}
	}
	
	/**
	 * Drives the robot forward AFTER a specified delay time.
	 * @param delayTimeMS The delay time before driving forward in milliseconds.
	 */
	public synchronized void driveForwardDelay(int delayTimeMS) {
		if (!this.isAlive()) {
			try {
				Thread.sleep(delayTimeMS);
			} catch (Exception e) {
				
			}
			driveForward();
		}
	}
	
	/**
	 * Reverse the output of the left side drive. Use this to correct if the robot is not driving forward properly. Positive output values should make the robot drive forward.
	 * @param reversed If true, the left side drive will be reversed.
	 */
	public void setLeftSideReversed(boolean reversed) {
		if (!this.isAlive())
			leftReversed = reversed;
	}
	
	/**
	 *  Reverse the output of the right side drive. Use this to correct if the robot is not driving forward properly. Positive output values should make the robot drive forward.
	 * @param reversed If true, the right side drive will be reversed.
	 */
	public void setRightSideReversed(boolean reversed) {
		if (!this.isAlive())
			rightReversed = reversed;
	}
	
	
	
	
	private void setDriveOutput(DriveSignal signal) {
		setDriveOutput(signal.getLeft(), signal.getRight());
	}
	
	private void setDriveOutput(double leftOutput, double rightOutput) {
		leftOutput = limit(leftOutput, 1);
		rightOutput = limit(rightOutput, 1);
		leftOutput = leftReversed ? -leftOutput : leftOutput;
		rightOutput = rightReversed ? -rightOutput : rightOutput;
		
		for (T t : leftDrive) {
			setSingleMotor(t, leftOutput);
		}
		for (T t : rightDrive) {
			setSingleMotor(t, rightOutput);
		}
	}
	
	private synchronized void setSingleMotor(T sc, double output) {
		if (sc instanceof SpeedController) {
			((SpeedController) sc).set(output);
		} else if (sc instanceof BaseMotorController) {
			((BaseMotorController) sc).set(ControlMode.PercentOutput, output);
		} else {
			System.out.println("Motor controller type not recognized!");
		}
	}
	
    private double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    private double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

	private ArrayList<T> leftDrive;
	private ArrayList<T> rightDrive;
	private ArrayList<CKAutoStep> autoSteps;
	
	private Robot robot;
	
	private CheesyDriveHelper cheesyDriveHelper;
	private boolean useCheesyDrive;
	
	private boolean leftReversed;
	private boolean rightReversed;
	
	private double timeoutStart;
	private double timeoutEnd;
	private int timeoutElapsedTimeMS;
	
	private static final int MIN_DRIVE_LOOP_TIME = 10;
	
	
	private class CKAutoStep {
		private double throttle;
		private double steer;
		private int holdTimeMS;
		
		public CKAutoStep(double throttle, double steer, int holdTimeMS) {
			this.throttle = throttle;
			this.steer = steer;
			this.holdTimeMS = holdTimeMS;
		}
		
		public double getThrottle() {
			return throttle;
		}
		public double getHeading() {
			return steer;
		}
		public int getHoldTimeMS() {
			return holdTimeMS;
		}
	}
	

	/**
	 * Helper class to implement "Cheesy Drive". "Cheesy Drive" simply means that the "turning" stick controls the curvature
	 * of the robot's path rather than its rate of heading change. This helps make the robot more controllable at high
	 * speeds. Also handles the robot's quick turn functionality - "quick turn" overrides constant-curvature turning for
	 * turn-in-place maneuvers.
	 */
	private class CheesyDriveHelper {

	    private static final double kThrottleDeadband = 0.02;
	    private static final double kWheelDeadband = 0.02;

	    // These factor determine how fast the wheel traverses the "non linear" sine curve.
	    private static final double kHighWheelNonLinearity = 0.65;
	    private static final double kLowWheelNonLinearity = 0.5;

	    private static final double kHighNegInertiaScalar = 4.0;

	    private static final double kLowNegInertiaThreshold = 0.65;
	    private static final double kLowNegInertiaTurnScalar = 3.5;
	    private static final double kLowNegInertiaCloseScalar = 4.0;
	    private static final double kLowNegInertiaFarScalar = 5.0;

	    private static final double kHighSensitivity = 0.95;
	    private static final double kLowSensitiity = 1.3;

	    private static final double kQuickStopDeadband = 0.2;
	    private static final double kQuickStopWeight = 0.1;
	    private static final double kQuickStopScalar = 5.0;

	    private double mOldWheel = 0.0;
	    private double mQuickStopAccumlator = 0.0;
	    private double mNegInertiaAccumlator = 0.0;

	    public DriveSignal cheesyDrive(double throttle, double wheel, boolean isQuickTurn, boolean isHighGear) {

	        wheel = handleDeadband(wheel, kWheelDeadband);
	        throttle = handleDeadband(throttle, kThrottleDeadband);
	        isQuickTurn |= throttle == 0;

	        double negInertia = wheel - mOldWheel;
	        mOldWheel = wheel;

	        double wheelNonLinearity;
	        if (isHighGear) {
	            wheelNonLinearity = kHighWheelNonLinearity;
	            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
	            // Apply a sin function that's scaled to make it feel better.
	            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
	            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
	        } else {
	            wheelNonLinearity = kLowWheelNonLinearity;
	            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
	            // Apply a sin function that's scaled to make it feel better.
	            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
	            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
	            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
	        }

	        double leftPwm, rightPwm, overPower;
	        double sensitivity;

	        double angularPower;
	        double linearPower;

	        // Negative inertia!
	        double negInertiaScalar;
	        if (isHighGear) {
	            negInertiaScalar = kHighNegInertiaScalar;
	            sensitivity = kHighSensitivity;
	        } else {
	            if (wheel * negInertia > 0) {
	                // If we are moving away from 0.0, aka, trying to get more wheel.
	                negInertiaScalar = kLowNegInertiaTurnScalar;
	            } else {
	                // Otherwise, we are attempting to go back to 0.0.
	                if (Math.abs(wheel) > kLowNegInertiaThreshold) {
	                    negInertiaScalar = kLowNegInertiaFarScalar;
	                } else {
	                    negInertiaScalar = kLowNegInertiaCloseScalar;
	                }
	            }
	            sensitivity = kLowSensitiity;
	        }
	        double negInertiaPower = negInertia * negInertiaScalar;
	        mNegInertiaAccumlator += negInertiaPower;

	        wheel = wheel + mNegInertiaAccumlator;
	        if (mNegInertiaAccumlator > 1) {
	            mNegInertiaAccumlator -= 1;
	        } else if (mNegInertiaAccumlator < -1) {
	            mNegInertiaAccumlator += 1;
	        } else {
	            mNegInertiaAccumlator = 0;
	        }
	        linearPower = throttle;

	        // Quickturn!
	        if (isQuickTurn) {
	            if (Math.abs(linearPower) < kQuickStopDeadband) {
	                double alpha = kQuickStopWeight;
	                mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator + alpha * limit(wheel, 1.0) * kQuickStopScalar;
	            }
	            overPower = 1.0;
	            angularPower = wheel;
	        } else {
	            overPower = 0.0;
	            angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
	            if (mQuickStopAccumlator > 1) {
	                mQuickStopAccumlator -= 1;
	            } else if (mQuickStopAccumlator < -1) {
	                mQuickStopAccumlator += 1;
	            } else {
	                mQuickStopAccumlator = 0.0;
	            }
	        }

	        rightPwm = leftPwm = linearPower;
	        leftPwm += angularPower;
	        rightPwm -= angularPower;

	        if (leftPwm > 1.0) {
	            rightPwm -= overPower * (leftPwm - 1.0);
	            leftPwm = 1.0;
	        } else if (rightPwm > 1.0) {
	            leftPwm -= overPower * (rightPwm - 1.0);
	            rightPwm = 1.0;
	        } else if (leftPwm < -1.0) {
	            rightPwm += overPower * (-1.0 - leftPwm);
	            leftPwm = -1.0;
	        } else if (rightPwm < -1.0) {
	            leftPwm += overPower * (-1.0 - rightPwm);
	            rightPwm = -1.0;
	        }
	        return new DriveSignal(leftPwm, rightPwm);
	    }

	    private double handleDeadband(double val, double deadband) {
	        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	    }
	    
	}
	
	private class DriveSignal {
	    private double mLeftMotor;
	    private double mRightMotor;
	    private boolean mBrakeMode;

	    public DriveSignal(double left, double right) {
	        this(left, right, false);
	    }

	    public DriveSignal(double left, double right, boolean brakeMode) {
	        mLeftMotor = left;
	        mRightMotor = right;
	        mBrakeMode = brakeMode;
	    }

	    public double getLeft() {
	        return mLeftMotor;
	    }

	    public double getRight() {
	        return mRightMotor;
	    }

	    public boolean getBrakeMode() {
	        return mBrakeMode;
	    }

	}

	
}

