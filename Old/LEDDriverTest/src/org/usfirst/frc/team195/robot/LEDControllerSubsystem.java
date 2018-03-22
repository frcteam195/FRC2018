package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.Timer;

import java.awt.*;
import java.util.List;

public class LEDControllerSubsystem extends Thread implements CustomSubsystem {
    private static final int MIN_LED_THREAD_LOOP_MS = 100;
    public static final int kDefaultBlinkCount = 4;
    public static final double kDefaultBlinkDuration = 0.2; // seconds for full cycle
    private static final double kDefaultTotalBlinkDuration = kDefaultBlinkCount * kDefaultBlinkDuration;

    private static LEDControllerSubsystem instance = null;
    private SystemState mSystemState = SystemState.OFF;
    private LEDState mRequestedState = LEDState.OFF;
    private boolean mIsLEDOn;
    private LEDDriverRGB mLED;
    private boolean runThread = true;
    private double mCurrentStateStartTime;
    private double mBlinkDuration;
    private int mBlinkCount;
    private double mTotalBlinkDuration;
    private ThreadRateControl threadRateControl = new ThreadRateControl();

    private LEDControllerSubsystem() throws Exception {
        mLED = LEDDriverRGB.getInstance();
        mLED.set(false);

        // Force a relay change.
        mIsLEDOn = true;
        setLEDOff();

        mBlinkDuration = kDefaultBlinkDuration;
        mBlinkCount = kDefaultBlinkCount;
        mTotalBlinkDuration = kDefaultTotalBlinkDuration;
    }

    public static LEDControllerSubsystem getInstance() {
        if(instance == null) {
            try {
                instance = new LEDControllerSubsystem();
            } catch (Exception ex) {
                //ConsoleReporter.report(ex, MessageLevel.DEFCON1);
            }
        }

        return instance;
    }

    public static LEDControllerSubsystem getInstance(List<CustomSubsystem> subsystemList) {
        subsystemList.add(getInstance());
        return instance;
    }

    @Override
    public void init() {
        mLED.setLEDColor(147, 0, 255);
    }

    @Override
    public void subsystemHome() {

    }

    @Override
    public void start() {
        mSystemState = SystemState.OFF;
        mRequestedState = LEDState.OFF;
        mLED.set(false);

        mCurrentStateStartTime = Timer.getFPGATimestamp();
        runThread = true;
        super.start();
    }

    @Override
    public void terminate() {
        ;
    }

    @Override
    public void run() {
        threadRateControl.start();

        while (runThread) {
            synchronized (LEDControllerSubsystem.this) {
                SystemState newState;
                double timeInState = Timer.getFPGATimestamp() - mCurrentStateStartTime;
                switch (mSystemState) {
                    case OFF:
                        newState = handleOff();
                        break;
                    case BLINKING:
                        newState = handleBlinking(timeInState);
                        break;
                    case FIXED_ON:
                        newState = handleFixedOn();
                        break;
                    default:
                        System.out.println("Fell through on LEDControllerSubsystem states!!");
                        newState = SystemState.OFF;
                }
                if (newState != mSystemState) {
                    //ConsoleReporter.report("LEDControllerSubsystem state " + mSystemState + " to " + newState, MessageLevel.INFO);
                    mSystemState = newState;
                    mCurrentStateStartTime = Timer.getFPGATimestamp();
                }
            }

            threadRateControl.doRateControl(MIN_LED_THREAD_LOOP_MS);
        }
    }

    public synchronized void setLEDColor(int redPWMOut, int greenPWMOut, int bluePWMOut) {
        mLED.setLEDColor(redPWMOut, greenPWMOut, bluePWMOut);
    }

    private SystemState defaultStateTransfer() {
        switch (mRequestedState) {
            case OFF:
                return SystemState.OFF;
            case BLINK:
                return SystemState.BLINKING;
            case FIXED_ON:
                return SystemState.FIXED_ON;
            default:
                return SystemState.OFF;
        }
    }

    private synchronized SystemState handleOff() {
        setLEDOff();
        return defaultStateTransfer();
    }

    private synchronized SystemState handleFixedOn() {
        setLEDOn();
        return defaultStateTransfer();
    }

    private synchronized SystemState handleBlinking(double timeInState) {
        if (timeInState > mTotalBlinkDuration) {
            setLEDOff();
            // Transition to OFF state and clear wanted state.
            mRequestedState = LEDState.OFF;
            return SystemState.OFF;
        }

        int cycleNum = (int) (timeInState / (mBlinkDuration / 2.0));
        if ((cycleNum % 2) == 0) {
            setLEDOn();
        } else {
            setLEDOff();
        }
        return SystemState.BLINKING;
    }

    public synchronized void setRequestedState(LEDState state) {
        mRequestedState = state;
    }

    private synchronized void setLEDOn() {
        if (!mIsLEDOn) {
            mIsLEDOn = true;
            mLED.set(true);
        }
    }

    private synchronized void setLEDOff() {
        if (mIsLEDOn) {
            mIsLEDOn = false;
            mLED.set(false);
        }
    }

    public synchronized void configureBlink(int blinkCount, double blinkDuration) {
        mBlinkDuration = blinkDuration;
        mBlinkCount = blinkCount;
        mTotalBlinkDuration = mBlinkCount * mBlinkDuration;
    }

    // Internal state of the system
    private enum SystemState {
        OFF, FIXED_ON, BLINKING
    }

    public enum LEDState {
        OFF, FIXED_ON, BLINK
    }
}
