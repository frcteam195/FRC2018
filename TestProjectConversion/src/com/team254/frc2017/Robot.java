package com.team254.frc2017;

import com.team254.frc2017.auto.modes.TestMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team254.frc2017.auto.AutoModeExecuter;
import com.team254.frc2017.loops.Looper;
import com.team254.frc2017.loops.RobotStateEstimator;
import com.team254.frc2017.paths.profiles.PathAdapter;
import com.team254.frc2017.subsystems.*;
import com.team254.lib.util.*;
import com.team254.lib.util.math.RigidTransform2d;

import java.util.Arrays;
import java.util.Map;

/**
 * The main robot class, which instantiates all robot parts and helper classes and initializes all loops. Some classes
 * are already instantiated upon robot startup; for those classes, the robot gets the instance as opposed to creating a
 * new object
 *
 * After initializing all robot parts, the code sets up the autonomous and teleoperated cycles and also code that runs
 * periodically inside both routines.
 *
 * This is the nexus/converging point of the robot code and the best place to start exploring.
 *
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {
    // Get subsystem instances
    private Drive mDrive = Drive.getInstance();
    //private Superstructure mSuperstructure = Superstructure.getInstance();
    //private MotorGearGrabber mGearGrabber = MotorGearGrabber.getInstance();
    //private LED mLED = LED.getInstance();
    private RobotState mRobotState = RobotState.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;

    // Create subsystem manager
//    private final SubsystemManager mSubsystemManager = new SubsystemManager(
//            Arrays.asList(Drive.getInstance(), Superstructure.getInstance(), Shooter.getInstance(),
//                    Feeder.getInstance(), Hopper.getInstance(), Intake.getInstance(),
//                    ConnectionMonitor.getInstance(), LED.getInstance(),
//                    MotorGearGrabber.getInstance()));

    private final SubsystemManager mSubsystemManager =
            new SubsystemManager(Arrays.asList(Drive.getInstance(), ConnectionMonitor.getInstance()));

    // Initialize other helper objects
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private ControlBoardInterface mControlBoard = ControlBoard.getInstance();

    private Looper mEnabledLooper = new Looper();

    //private VisionServer mVisionServer = VisionServer.getInstance();

    //private AnalogInput mCheckLightButton = new AnalogInput(Constants.kLEDOnId);

    //private DelayedBoolean mDelayedAimButton;

    //private LatchedBoolean mCommitTuning = new LatchedBoolean();
    //private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mTuningFlywheelMap = new InterpolatingTreeMap<>();

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    public void zeroAllSensors() {
        mSubsystemManager.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
        mDrive.zeroSensors();
    }

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            //mEnabledLooper.register(VisionProcessor.getInstance());
            mEnabledLooper.register(RobotStateEstimator.getInstance());

            //mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());

            //AutoModeSelector.initAutoModeSelector();

            //mDelayedAimButton = new DelayedBoolean(Timer.getFPGATimestamp(), 0.1);
            // Force an true update now to prevent robot from running at start.
            //mDelayedAimButton.update(Timer.getFPGATimestamp(), true);

            // Pre calculate the paths we use for auto.
            PathAdapter.calculatePaths();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        zeroAllSensors();
    }

    /**
     * Initializes the robot for the beginning of autonomous mode (set drivebase, intake and superstructure to correct
     * states). Then gets the correct auto mode from the AutoModeSelector
     *
     * @see AutoModeSelector.java
     */
    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

            System.out.println("Auto start timestamp: " + Timer.getFPGATimestamp());

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }

            zeroAllSensors();
//            mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
//            mSuperstructure.setActuateHopper(false);
//            mSuperstructure.setOverrideCompressor(true);

            mAutoModeExecuter = null;

            //Intake.getInstance().reset();

            // Shift to high
            mDrive.setHighGear(true);
            mDrive.setBrakeMode(true);

            mEnabledLooper.start();
            //mSuperstructure.reloadConstants();
            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(new TestMode());
            mAutoModeExecuter.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        allPeriodic();
    }

    /**
     * Initializes the robot for the beginning of teleop
     */
    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();

            // Start loopers
            mEnabledLooper.start();
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
            mDrive.setBrakeMode(false);
            // Shift to high
            mDrive.setHighGear(true);
            zeroAllSensors();
            //mSuperstructure.reloadConstants();
            //mSuperstructure.setOverrideCompressor(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
     *
     * The code uses state machines to ensure that no matter what buttons the driver presses, the robot behaves in a
     * safe and consistent manner.
     *
     * Based on driver input, the code sets a desired state for each subsystem. Each subsystem will constantly compare
     * its desired and actual states and act to bring the two closer.
     */
    @Override
    public void teleopPeriodic() {
        try {
            double timestamp = Timer.getFPGATimestamp();
            // Drive base
            double throttle = mControlBoard.getThrottle();
            double turn = mControlBoard.getTurn();

            //mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
             //       !mControlBoard.getLowGear()));
            DriveSignal ds = mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                    !mControlBoard.getLowGear());
            mDrive.setVelocitySetpoint(-ds.getLeft() * 100.0, -ds.getRight() * 100.0);
            boolean wantLowGear = mControlBoard.getLowGear();
            mDrive.setHighGear(!wantLowGear);

            allPeriodic();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;

            mEnabledLooper.stop();

            // Call stop on all our Subsystems.
            mSubsystemManager.stop();

            mDrive.setOpenLoop(DriveSignal.NEUTRAL);

            PathAdapter.calculatePaths();

            // If are tuning, dump map so far.
//            if (Constants.kIsShooterTuning) {
//                for (Map.Entry<InterpolatingDouble, InterpolatingDouble> entry : mTuningFlywheelMap.entrySet()) {
//                    System.out.println("{" +
//                            entry.getKey().value + ", " + entry.getValue().value + "},");
//                }
//            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        final double kVoltageThreshold = 0.15;
//        if (mCheckLightButton.getAverageVoltage() < kVoltageThreshold) {
//            mLED.setLEDOn();
//        } else {
//            mLED.setLEDOff();
//        }

        zeroAllSensors();
        allPeriodic();
    }

    @Override
    public void testInit() {
        Timer.delay(0.5);

        boolean results = Drive.getInstance().checkSystem();
//        results &= Feeder.getInstance().checkSystem();
//        results &= Intake.getInstance().checkSystem();
//        results &= MotorGearGrabber.getInstance().checkSystem();
//        results &= Shooter.getInstance().checkSystem();
//        results &= Hopper.getInstance().checkSystem();

        if (!results) {
            System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
        } else {
            System.out.println("ALL SYSTEMS PASSED");
        }
    }

    @Override
    public void testPeriodic() {
    }

    /**
     * Helper function that is called in all periodic functions
     */
    public void allPeriodic() {
        mRobotState.outputToSmartDashboard();
        mSubsystemManager.outputToSmartDashboard();
        mSubsystemManager.writeToLog();
        mEnabledLooper.outputToSmartDashboard();
        //SmartDashboard.putBoolean("camera_connected", mVisionServer.isConnected());

        ConnectionMonitor.getInstance().setLastPacketTime(Timer.getFPGATimestamp());
    }
}
