package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Logging extends SubsystemBase {
    private final DataLog m_log0 = DataLogManager.getLog();
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable evenTable = inst.getTable("EvenLog");

    // Enable logging switch
    private final BooleanTopic doLoggingNT = evenTable.getBooleanTopic("DriveState/doLogging");
    private final BooleanPublisher doLoggingNTPub = doLoggingNT.publish();
    private final BooleanSubscriber doLoggingNTSub = doLoggingNT.subscribe(false);
    private boolean doLogging = false; // the real value that gets payed attention to, as the drivers switch is locked to true during a match


    // Drivetrain logging/telemetry objects
    private final DoubleArrayLogEntry chassisPoseLog = new DoubleArrayLogEntry(m_log0, "DriveState/chassisPose");
    private final StructPublisher<Pose2d> chassisPoseNT = evenTable.getStructTopic("DriveState/chassisPose", Pose2d.struct).publish();

    private final DoubleArrayLogEntry chassisVelocityLog = new DoubleArrayLogEntry(m_log0, "DriveState/chassisVelocity");
    private final StructPublisher<ChassisSpeeds> chassisVelocityNT = evenTable.getStructTopic("DriveState/chassisVelocity", ChassisSpeeds.struct).publish();


    // Drivetrain Pose
    private final DoubleArrayPublisher fieldPub = evenTable.getDoubleArrayTopic("DriveState/Pose/Robot")
        .publish(); // If its not called exactly `robot` elastic wont render it as the big one
    private final StringPublisher fieldTypePub = evenTable.getStringTopic("DriveState/Pose/.type")
        .publish(); // Tells Elastic how to render it
    private final double[] poseArray = new double[3];

    // Match time and shifts
    private final DoublePublisher matchTimePub = evenTable.getDoubleTopic("MatchTime").publish();
    private final DoublePublisher shiftTimePub = evenTable.getDoubleTopic("ShiftTime").publish();
    private double shiftTime;
    private double matchTime;

    // Who won auton?
    // TODO: Move a lot of crap to constants file
    private final StringArrayPublisher autonWinner = evenTable.getStringArrayTopic("AutonWinner").publish();
    private final String[] autonWinnerColorNone = new String[] {"#FF0000", "#0000FF"};
    private final String[] autonWinnerColorError = new String[] {"#00FF00", "#00FF00"};
    private final String[] autonWinnerColorRed = new String[] {"#FF0000", "#000000"};
    private final String[] autonWinnerColorBlue = new String[] {"#000000", "#0000FF"};


    // YUP WE DOING SIGNLETONS LET IT RIDE
    private static Logging instance;

    public static Logging getLTInstance() {
        if (instance != null) {
            return instance;
        } else {
            return new Logging();
        }
    }

    public Logging() {
        // Initialize the doLogging switch
        doLoggingNTPub.set(doLogging);
    }

    public NetworkTable getRootTable() {
        return evenTable;
    }

    @Override
    public void periodic() {
        /* Should we be logging? Ask the driver/programmer 
            unless FMS is attatched. Logging must be done if it is
        */
        if (DriverStation.isFMSAttached()) {
            doLogging = true;
            doLoggingNTPub.set(doLogging);
        } else {
            doLogging = doLoggingNTSub.get();
        }

        // Give driver match time
        matchTime = DriverStation.getMatchTime();
        matchTimePub.set(matchTime);
        if (DriverStation.isAutonomous()) {
            shiftTimePub.set(matchTime);
        } else {
            if (matchTime > 130) {
                shiftTimePub.set(matchTime-130);
            } else if (matchTime > 105) {
                shiftTimePub.set(matchTime-105);
            } else if (matchTime > (80)) {
                shiftTimePub.set(matchTime-80);
            } else if (matchTime > (55)) {
                shiftTimePub.set(matchTime-55);
            } else if (matchTime > (30)) {
                shiftTimePub.set(matchTime-30);
            } else { 
                shiftTimePub.set(-3100);
            }
        }
        // Give the driver shift time
        composeGameMessage();
    }

    public void composeGameMessage() {
        String gameData = DriverStation.getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B':
                    autonWinner.set(autonWinnerColorBlue);
                break;
                case 'R':
                    autonWinner.set(autonWinnerColorRed);
                break;
                default:
                    autonWinner.set(autonWinnerColorError);
                break;
            }
        } else {
            autonWinner.set(autonWinnerColorNone);
        }
    }

    public void logCTREChassis(SwerveDriveState state) {
        if (doLogging) {
            chassisPoseLog.append(new double[] {
                state.Pose.getX(), 
                state.Pose.getY(),
                state.Pose.getRotation().getDegrees()
            });

            chassisVelocityLog.append(new double[] {
                state.Speeds.vxMetersPerSecond,
                state.Speeds.vyMetersPerSecond,
                state.Speeds.omegaRadiansPerSecond * 9.549297 // Gets in Rotations Per Minute
            });
        }

        chassisPoseNT.set(state.Pose);
        chassisVelocityNT.set(state.Speeds);

        fieldTypePub.set("Field2d");

        poseArray[0] = state.Pose.getX();
        poseArray[1] = state.Pose.getY();
        poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(poseArray);
    }
}
