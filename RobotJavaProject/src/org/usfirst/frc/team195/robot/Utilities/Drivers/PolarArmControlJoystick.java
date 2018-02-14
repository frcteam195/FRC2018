package org.usfirst.frc.team195.robot.Utilities.Drivers;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm.PolarCoordinate;
import org.usfirst.frc.team195.robot.Utilities.QuickMaths;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

public class PolarArmControlJoystick extends KnightJoystick {
	private double minRadius;
	private double maxRadius;
	private double minTheta;
	private double maxTheta;

	private double mRadiusVal = 0;
	private double mThetaVal = 0;

	private int yAxisID = 1;
	private int zAxisID = 2;

	private double mPrevTime = -1;

	private double factorInchesPerSec;
	private double factorDegPerSec;

	private double kJoystickYDeadband = Constants.kArmJoystickYDeadband;
	private double kJoystickZDeadband = Constants.kArmJoystickZDeadband;

	public PolarArmControlJoystick(int port, double minRadius, double maxRadius, double minTheta, double maxTheta, double inchesPerSec, double degPerSec) {
		super(port);
		this.minRadius = minRadius;
		this.maxRadius = maxRadius;
		this.minTheta = minTheta;
		this.maxTheta = maxTheta;
		this.factorInchesPerSec = inchesPerSec;
		this.factorDegPerSec = degPerSec;
	}

	public void setYAxisID(int yAxisID) {
		this.yAxisID = yAxisID;
	}

	public void setZAxisID(int zAxisID) {
		this.zAxisID = zAxisID;
	}

	public synchronized void start() {
		mPrevTime = Timer.getFPGATimestamp();
	}

	public synchronized PolarCoordinate getPolarMappingFromJoystick() {
		double currTime = Timer.getFPGATimestamp();
		double dt = currTime - mPrevTime;

		if (mPrevTime == -1) {
			ConsoleReporter.report("Call start first for the joystick!");
			mPrevTime = currTime;
			return null;
		}

		mRadiusVal += (QuickMaths.normalizeJoystickWithDeadband(-getRawAxis(yAxisID), kJoystickYDeadband)) * dt * factorInchesPerSec;
		mThetaVal += (QuickMaths.normalizeJoystickWithDeadband(-getRawAxis(zAxisID), kJoystickZDeadband)) * dt * factorDegPerSec;

		mPrevTime = currTime;

		mRadiusVal = Util.limit(mRadiusVal, minRadius, maxRadius);
		mThetaVal = Util.limit(mThetaVal, minTheta, maxTheta);

		return new PolarCoordinate(mRadiusVal, mThetaVal);
	}

}
