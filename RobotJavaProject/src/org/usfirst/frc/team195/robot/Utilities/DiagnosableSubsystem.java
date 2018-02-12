package org.usfirst.frc.team195.robot.Utilities;

public interface DiagnosableSubsystem {
	/**
	 * Method to diagnose a subsystem
	 * @return Returns true if the system passes all tests
	 */
	public boolean runDiagnostics();
}
