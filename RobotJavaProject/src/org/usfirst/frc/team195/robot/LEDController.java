package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.Drivers.LEDDriver;
import org.usfirst.frc.team195.robot.Utilities.Drivers.LEDDriverCANifier;

import java.util.Arrays;
import java.util.LinkedList;

public class LEDController extends Thread {
    private static final int MIN_LED_THREAD_LOOP_MS = 50;
    public static final int kDefaultBlinkCount = 6;
    public static final double kDefaultBlinkDuration = 0.2; // seconds for full cycle
    private static final double kDefaultTotalBlinkDuration = kDefaultBlinkCount * kDefaultBlinkDuration;
	private static final double kSlowBlinkDivisor = 1.5;
	private static final double kFastBlinkDivisor = 3;
	private static final double kLetterPause = 0.5;
	private static final double kWordPause = 0.75;

	private static LEDController instance = null;
    private LEDState mDefaultState = LEDState.FIXED_ON;
    private SystemState mSystemState = SystemState.OFF;
    private LEDState mRequestedState = LEDState.OFF;
    private boolean mIsLEDOn;
    private LEDDriver mLED;
    private boolean runThread = true;
    private double mCurrentStateStartTime;
    private double mBlinkDuration;
    private int mBlinkCount;
    private double mTotalBlinkDuration;
    private ThreadRateControl threadRateControl = new ThreadRateControl();
    private MorseCodeTranslator morseCodeTranslator = new MorseCodeTranslator();
    private LinkedList<String> requestedMorseMessage;
    private LinkedList<String> runningMorseMessage;
    private double mPrevMorseTime;
    private double mMorseTransitionTime = 0.5;
    private MorseState mMorseState = MorseState.LOAD;
    private double morseStateTime = 0;
    private Character currentMorseChar = ' ';
    private LinkedList<Character> currentMorseCode;
    private String mPrevMessage = "";

    private LEDController() throws Exception {
    	super();
		super.setPriority(Constants.kLEDThreadPriority);
        mLED = new LEDDriverCANifier(Controllers.getInstance().getCANifierLED());
        mLED.set(false);

        // Force a relay change.
        mIsLEDOn = true;
        setLEDOff();

        setLEDColor(Constants.kDefaultColor);
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
					case MORSE:
						newState = handleMorse();
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
			case MORSE:
				return SystemState.MORSE;
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

	private synchronized SystemState handleMorse() {

		switch (mMorseState) {
			case LOAD:
				if (requestedMorseMessage == null || requestedMorseMessage.size() <= 0) {
					return returnOffMorse();
				}
				runningMorseMessage = new LinkedList<String>(requestedMorseMessage);
				if (runningMorseMessage.size() > 0) {
					setLEDOff();
					mMorseState = MorseState.NEXT_CHAR;
				} else {
					return returnOffMorse();
				}
				break;
			case NEXT_CHAR:
				if (runningMorseMessage != null && runningMorseMessage.peek() != null) {
					currentMorseCode = new LinkedList<Character>(Arrays.asList(runningMorseMessage.poll().chars().mapToObj(c -> (char)c).toArray(Character[]::new)));
					mMorseState = MorseState.NEXT_MORSE_CODE;
				} else {
					mMorseState = MorseState.DONE;
					morseStateTime = Timer.getFPGATimestamp();
				}
				break;
			case NEXT_MORSE_CODE:
				if (currentMorseCode != null) {
					if (currentMorseCode.peek() != null) {
						currentMorseChar = currentMorseCode.poll();
						mMorseState = MorseState.BLINK_ON;
						morseStateTime = Timer.getFPGATimestamp();
					} else {
						//Delay to delineate between letters in a string
						if ((Timer.getFPGATimestamp() - morseStateTime) > kLetterPause) {
							mMorseState = MorseState.NEXT_CHAR;
							//morseStateTime = Timer.getFPGATimestamp();
						}
					}
				} else {
					return returnOffMorse();
				}
				break;
			case BLINK_ON:
				if (currentMorseChar == '.' || currentMorseChar == '-') {
					double divisorVal = currentMorseChar == '.' ? kFastBlinkDivisor : kSlowBlinkDivisor;
					setLEDOn();
					if ((Timer.getFPGATimestamp() - morseStateTime) > mBlinkDuration / divisorVal) {
						mMorseState = MorseState.BLINK_OFF;
						morseStateTime = Timer.getFPGATimestamp();
					}
				} else {
					mMorseState = MorseState.NEXT_MORSE_CODE;
				}
				break;
			case BLINK_OFF:
				if (currentMorseChar == '.' || currentMorseChar == '-') {
					double divisorVal = currentMorseChar == '.' ? kFastBlinkDivisor : kSlowBlinkDivisor;
					setLEDOff();
					if ((Timer.getFPGATimestamp() - morseStateTime) > mBlinkDuration / divisorVal) {
						mMorseState = MorseState.NEXT_MORSE_CODE;
						currentMorseChar = ' ';
						morseStateTime = Timer.getFPGATimestamp();
					}
				} else {
					mMorseState = MorseState.NEXT_MORSE_CODE;
				}
				break;
			case DONE:
				if ((Timer.getFPGATimestamp() - morseStateTime) > kWordPause) {
					return returnOffMorse();
				}
				break;
			default:
				return returnOffMorse();
		}

		return SystemState.MORSE;
	}

    public synchronized void setRequestedState(LEDState state) {
        mRequestedState = state;
    }

    private synchronized SystemState returnOffMorse() {
		mMorseState = MorseState.LOAD;
		morseStateTime = 0;
		currentMorseChar = ' ';
		//setLEDDefaultState();
		setLEDOff();
		//ConsoleReporter.report("Off morse called.");
		return SystemState.OFF;
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
		mLED.setLEDColor(rgbColor);
    }

    public synchronized void setLEDColor(int redPWMOut, int greenPWMOut, int bluePWMOut) {
        setLEDColor(new RGBColor(redPWMOut, greenPWMOut, bluePWMOut));
    }

    public synchronized void setLEDDefaultState(LEDState defaultState) {
        this.mDefaultState = defaultState;
    }

    public synchronized void setMessage(String message) {
		setMessage(message, false);
	}

	public synchronized void setMessage(String message, boolean autoStartMessage) {
    	if (!message.equalsIgnoreCase(mPrevMessage)) {
			this.requestedMorseMessage = morseCodeTranslator.getMorseCode(message);
			mPrevMessage = message;
		}

		if (autoStartMessage)
			setRequestedState(LEDState.MORSE);
	}

    // Internal state of the system
    private enum SystemState {
        OFF, FIXED_ON, BLINKING, MORSE
    }

    private enum MorseState {
    	LOAD, BLINK_ON, BLINK_OFF, NEXT_CHAR, NEXT_MORSE_CODE, DONE
	}

    public enum LEDState {
        OFF, FIXED_ON, BLINK, MORSE
    }
}
