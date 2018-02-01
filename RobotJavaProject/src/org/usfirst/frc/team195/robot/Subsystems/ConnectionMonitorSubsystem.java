package org.usfirst.frc.team195.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.CustomSubsystem;
import org.usfirst.frc.team195.robot.Utilities.ThreadRateControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.LatchedBoolean;

import java.util.List;

/**
 * Keeps track of the robot's connection to the driver station. If it disconnects for more than 1 second, start blinking
 * the LEDs.
 */
public class ConnectionMonitorSubsystem extends Thread implements CustomSubsystem {

    private static final int MIN_CONNECTION_MONITOR_THREAD_LOOP_MS = 500;
    private static ConnectionMonitorSubsystem instance = null;
    private boolean hasConnection;
    private boolean runThread;
    private double mLastPacketTime;
    private LatchedBoolean mJustReconnected;
    private LatchedBoolean mJustDisconnected;
    private LEDControllerSubsystem mLED;
    private ThreadRateControl threadRateControl = new ThreadRateControl();

    private ConnectionMonitorSubsystem() throws Exception {
        mLastPacketTime = 0.0;
        mJustReconnected = new LatchedBoolean();
        mJustDisconnected = new LatchedBoolean();
        runThread = false;
        hasConnection = true;
        mLED = LEDControllerSubsystem.getInstance();
    }

    public static ConnectionMonitorSubsystem getInstance() {
        if(instance == null) {
            try {
                instance = new ConnectionMonitorSubsystem();
            } catch (Exception ex) {
                ConsoleReporter.report(ex, MessageLevel.DEFCON1);
            }
        }

        return instance;
    }

    public static ConnectionMonitorSubsystem getInstance(List<CustomSubsystem> subsystemList) {
        subsystemList.add(getInstance());
        return instance;
    }

    @Override
    public void start() {
        mLastPacketTime = Timer.getFPGATimestamp();
        runThread = true;
        super.start();
    }

    @Override
    public void run() {
        threadRateControl.start();

        while (runThread) {
            hasConnection = DriverStation.getInstance().waitForData(1);

            if (hasConnection)
                mLastPacketTime = Timer.getFPGATimestamp();
            else
                mLED.setRequestedState(LEDControllerSubsystem.LEDState.BLINK);

            if (mJustReconnected.update(hasConnection))
                justReconnected();

            if (mJustDisconnected.update(!hasConnection))
                justDisconnected();

            threadRateControl.doRateControl(MIN_CONNECTION_MONITOR_THREAD_LOOP_MS);
        }
    }

    public boolean isConnected() {
        return hasConnection;
    }

    public double getLastPacketTime() {
        return mLastPacketTime;
    }

    private void justReconnected() {
        // Reconfigure blink if we are just connected.
        mLED.configureBlink(LEDControllerSubsystem.kDefaultBlinkCount, LEDControllerSubsystem.kDefaultBlinkDuration);
    }

    private void justDisconnected() {
        // Reconfigure blink if we are just disconnected.
        mLED.configureBlink(LEDControllerSubsystem.kDefaultBlinkCount, LEDControllerSubsystem.kDefaultBlinkDuration * 2.0);
    }

    @Override
    public void init() {

    }

    @Override
    public void subsystemHome() {

    }

    @Override
    public void terminate() {
        ConsoleReporter.report("CAN'T STOP, WON'T STOP, DON'T CALL ME!", MessageLevel.ERROR);
    }
}
