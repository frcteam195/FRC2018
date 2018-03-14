package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.RGBColor;
import org.usfirst.frc.team195.robot.Utilities.ThreadRateControl;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.LatchedBoolean;

/**
 * Keeps track of the robot's connection to the driver station. If it disconnects for more than 1 second, start blinking
 * the LEDs.
 */
public class ConnectionMonitor extends Thread {

    private static final int MIN_CONNECTION_MONITOR_THREAD_LOOP_MS = 500;
    private static ConnectionMonitor instance = null;
    private boolean hasConnection;
    private boolean runThread;
    private double mLastPacketTime;
    private LatchedBoolean mJustReconnected;
    private LatchedBoolean mJustDisconnected;
    private LEDController mLED;
    private ThreadRateControl threadRateControl = new ThreadRateControl();

    private ConnectionMonitor() throws Exception {
        super();
        super.setPriority(Constants.kConnectionMonitorThreadPriority);
        mLastPacketTime = 0.0;
        mJustReconnected = new LatchedBoolean();
        mJustDisconnected = new LatchedBoolean();
        runThread = false;
        hasConnection = true;
        mLED = LEDController.getInstance();
    }

    public static ConnectionMonitor getInstance() {
        if(instance == null) {
            try {
                instance = new ConnectionMonitor();
            } catch (Exception ex) {
                ConsoleReporter.report(ex, MessageLevel.DEFCON1);
            }
        }

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

            if (hasConnection) {
                mLastPacketTime = Timer.getFPGATimestamp();
            }
            else {
                mLED.setLEDColor(new RGBColor(255, 0, 0));
                mLED.setMessage("sos", true);
                //mLED.setRequestedState(LEDController.LEDState.BLINK);
            }

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
        mLED.setLEDColor(Constants.kDefaultColor);
        mLED.configureBlink(LEDController.kDefaultBlinkCount, LEDController.kDefaultBlinkDuration);
    }

    private void justDisconnected() {
        // Reconfigure blink if we are just disconnected.
        mLED.configureBlink(LEDController.kDefaultBlinkCount, LEDController.kDefaultBlinkDuration * 2.0);
    }

}
