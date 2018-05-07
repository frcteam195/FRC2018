package org.usfirst.frc.team195.robot.Actions.Framework;

import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;

/**
 * Runnable class with reports all uncaught throws to CrashTracker
 */
public abstract class CrashTrackingRunnable implements Runnable {

    @Override
    public final void run() {
        try {
            runCrashTracked();
        } catch (Throwable t) {
            ConsoleReporter.report(t);
        }
    }

    public abstract void runCrashTracked();
}
