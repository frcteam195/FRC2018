package org.usfirst.frc.team195.robot.Actions.Framework;

/**
 * Template action for something that only needs to be done once and has no need for updates.
 * 
 * @see Action
 */
public abstract class RunOnceAction implements Action {
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        runOnce();
    }

    public abstract void runOnce();
}
