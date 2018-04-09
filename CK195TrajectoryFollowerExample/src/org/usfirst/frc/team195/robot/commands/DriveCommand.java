package org.usfirst.frc.team195.robot.commands;

//import RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team195.robot.Robot;

/**
 *
 */
public class DriveCommand extends Command {
    public DriveCommand() {
        // Use requires() here to declare subsystem dependencies
       requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.print("initialized drive w joy");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	Robot.drive.drive_w_j();

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
