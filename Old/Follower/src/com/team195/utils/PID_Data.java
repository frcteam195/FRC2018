package com.team195.utils;

import edu.wpi.first.wpilibj.Timer;

public class PID_Data {
	private double kP;
	private double kI;
	private double kD;
	private double error;
	private double lastError;
	private double errorSum;
	private double errorD;
	private double output;
	private double currTime = 0;
	private double prevTime = 0;
	private double timeAccum = 0;

	public PID_Data(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		error = 0.0;
		lastError = 0.0;
		errorSum = 0.0;
		errorD = 0.0;
		output = 0.0;
	}

	public synchronized void calculateOutput(double setpoint, double actual) {
		currTime = Timer.getFPGATimestamp();

		double dt = currTime - prevTime;

		error = setpoint - actual;
		errorSum += (error*dt);
		errorD = (error - lastError) / dt;
		lastError = error;
		output = kP * error + kI * errorSum + kD * errorD;

		prevTime = currTime;
	}

	public synchronized void resetIAccum() { errorSum = 0; }

	public synchronized double getOutput() {
		return output;
	}

	public synchronized double getReversedOutput() {
		return -output;
	}
}
