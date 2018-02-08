package org.usfirst.frc.team195.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.Drivers.LEDDriverRGB;

public class LEDController extends Thread {
    private static final int MIN_LED_THREAD_LOOP_MS = 100;
    public static final int kDefaultBlinkCount = 6;
    public static final double kDefaultBlinkDuration = 0.2; // seconds for full cycle
    private static final double kDefaultTotalBlinkDuration = kDefaultBlinkCount * kDefaultBlinkDuration;
    private static final RGBColor kDefaultColor = new RGBColor(210, 0, 255);  //Default purple color

    private static LEDController instance = null;
    private LEDState mDefaultState = LEDState.FIXED_ON;
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

    private LEDController() throws Exception {
        mLED = new LEDDriverRGB(Controllers.getInstance().getRedLED(), Controllers.getInstance().getGreenLED(), Controllers.getInstance().getBlueLED());
        mLED.set(false);

        // Force a relay change.
        mIsLEDOn = true;
        setLEDOff();

        setLEDColor(kDefaultColor);
        mBlinkDuration = kDefaultBlinkDuration;
        mBlinkCount = kDefaultBlinkCount;
        mTotalBlinkDuration = kDefaultTotalBlinkDuration;
    }

    public static LEDController getInstance() {
        if(instance == null) {
            try {
                instance = new LEDController();
            } catch (Exception ex) {
                ConsoleReporter.report(ex, MessageLevel.DEFCON1);
            }
        }

        return instance;
    }

    @Override
	public void start() {
		mSystemState = SystemState.OFF;
		mLED.set(false);

		mCurrentStateStartTime = Timer.getFPGATimestamp();

    	runThread = true;
    	if (!super.isAlive())
    		super.start();
	}

    @Override
	public void run() {
		threadRateControl.start();
    	while (runThread) {
			synchronized (LEDController.this) {
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
						ConsoleReporter.report("Fell through on LEDController states!!", MessageLevel.ERROR);
						newState = SystemState.OFF;
				}
				if (newState != mSystemState) {
					ConsoleReporter.report("LEDController state " + mSystemState + " to " + newState, MessageLevel.INFO);
					mSystemState = newState;
					mCurrentStateStartTime = Timer.getFPGATimestamp();
				}
			}
			threadRateControl.doRateControl(MIN_LED_THREAD_LOOP_MS);
		}
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
            //setLEDOff();
            // Transition to OFF state and clear wanted state.
            //setRequestedState(LEDState.OFF);
            setLEDDefaultState();
            setLEDOff();
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

    private synchronized void setLEDDefaultState() {
        setRequestedState(mDefaultState);
    }

    public synchronized void configureBlink(int blinkCount, double blinkDuration) {
        mBlinkDuration = blinkDuration;
        mBlinkCount = blinkCount;
        mTotalBlinkDuration = mBlinkCount * mBlinkDuration;
    }

    public synchronized void setLEDColor(RGBColor rgbColor) {
        setLEDColor(rgbColor.red, rgbColor.green, rgbColor.blue);
    }

    public synchronized void setLEDColor(int redPWMOut, int greenPWMOut, int bluePWMOut) {
        mLED.setLEDColor(redPWMOut, greenPWMOut, bluePWMOut);
    }

    public synchronized void setLEDDefaultState(LEDState defaultState) {
        this.mDefaultState = defaultState;
    }

    // Internal state of the system
    private enum SystemState {
        OFF, FIXED_ON, BLINKING
    }

    public enum LEDState {
        OFF, FIXED_ON, BLINK
    }
}
