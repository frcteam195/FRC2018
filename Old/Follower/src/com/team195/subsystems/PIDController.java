package com.team195.subsystems;

import com.team195.Robot;
import com.team195.RobotMap;
import com.team195.utils.*;

import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

public class PIDController extends Thread {
	private static PIDController instance = null;

	private AHRS navx;

	private PID_Data anglePID;
	private PID_Data distancePID;

	private double leftOutput;
	private double rightOutput;
	private double finalLeftOutput;
	private double finalRightOutput;
	private double leftAngle;
	private double rightAngle;
	private double distance;
	private ThreadRateControl trc = new ThreadRateControl();


	private PIDController() {
		navx = new AHRS(SPI.Port.kMXP);

		anglePID = new PID_Data(RobotMap.ANGLE_PID_KP, RobotMap.ANGLE_PID_KI, RobotMap.ANGLE_PID_KD);
		distancePID = new PID_Data(RobotMap.DRIVE_PID_KP, RobotMap.DRIVE_PID_KI, RobotMap.DRIVE_PID_KD);

		leftOutput = 0.0;
		rightOutput = 0.0;
		finalLeftOutput = 0.0;
		finalRightOutput = 0.0;
	}

	public static PIDController getInstance() {
		if (instance == null)
			instance = new PIDController();

		return instance;
	}

	@Override
	public void run() {
		trc.start();
		while (true) {
			calculateOutputs();
			normalizeOutputs();

			try {
				trc.doRateControl(10);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	private void calculateOutputs() {

		if (Math.abs(navx.getYaw() - VisionData.deviation) > RobotMap.allowedDeviation) {
			anglePID.calculateOutput(navx.getYaw() + VisionData.deviation, navx.getYaw());
			leftAngle = anglePID.getOutput();
			rightAngle = anglePID.getReversedOutput();
		}
		else {
			leftAngle = 0;
			rightAngle = 0;
			//anglePID.resetIAccum();
		}
		if (Math.abs(RobotMap.DESIRED_DISTANCE - VisionData.distance) > RobotMap.allowedDistanceError) {
			distancePID.calculateOutput(RobotMap.DESIRED_DISTANCE, VisionData.distance);
			distance = distancePID.getReversedOutput();
		}
		else {
			distance = 0;
			//distancePID.resetIAccum();
		}

	}

	private void normalizeOutputs() {
		leftOutput = (leftAngle + distance);
		rightOutput = (rightAngle + distance);

		finalLeftOutput = Math.tanh(leftOutput/40);
		finalRightOutput = Math.tanh(rightOutput/40);

		//System.out.println(leftOutput + "," + rightOutput);
	}

	public synchronized void setOutputs() {
		//System.out.println(VisionData.targetFound);
		if (VisionData.targetFound) {
			Drive.getInstance().setDrive(finalLeftOutput, finalRightOutput);
		}
		else {
			Drive.getInstance().setDrive(0, 0);
		}
	}
}
