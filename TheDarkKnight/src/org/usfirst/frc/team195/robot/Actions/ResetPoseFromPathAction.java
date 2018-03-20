package org.usfirst.frc.team195.robot.Actions;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Actions.Framework.RunOnceAction;
import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathFollowerRobotState;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.RigidTransform2d;

/**
 * Resets the robot's current pose based on the starting pose stored in the pathContainer object.
 *
 */
public class ResetPoseFromPathAction extends RunOnceAction {

    protected PathContainer mPathContainer;

    public ResetPoseFromPathAction(PathContainer pathContainer) {
        mPathContainer = pathContainer;
    }

    @Override
    public synchronized void runOnce() {
        RigidTransform2d startPose = mPathContainer.getStartPose();
        PathFollowerRobotState.getInstance().reset(Timer.getFPGATimestamp(), startPose);
        DriveBaseSubsystem.getInstance().setGyroAngle(startPose.getRotation());
    }
}
