package org.usfirst.frc.team195.robot.Actions.Framework;

import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;

/**
 * Waits for the robot to pass by a provided path marker (i.e. a waypoint on the field). This action routinely compares
 * to the crossed path markers provided by the drivetrain (in Path Control mode) and returns if the parameter path
 * marker is inside the drivetrain's Path Markers Crossed list
 *
 *            Path Marker to determine if crossed
 */
public class WaitForElevatorDownAction implements Action {

	private CubeHandlerSubsystem cubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();
	private double elevatorHeight;

	public WaitForElevatorDownAction(double elevatorHeight) {
		this.elevatorHeight = elevatorHeight;
	}

	@Override
	public boolean isFinished() {
		return cubeHandlerSubsystem.getElevatorHeight() < elevatorHeight;
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
	}

	@Override
	public void start() {
	}

}