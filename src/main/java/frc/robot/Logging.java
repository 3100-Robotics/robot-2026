package frc.robot;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

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
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Logging extends SubsystemBase {
    private final DataLog m_log0 = DataLogManager.getLog();
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable evenTable = inst.getTable("EvenLog");

    // Enable logging switch
    private final BooleanTopic doLoggingNT = evenTable.getBooleanTopic("DriveState/doLogging");
    private final BooleanPublisher doLoggingNTPub = doLoggingNT.publish();
    private final BooleanSubscriber doLoggingNTSub = doLoggingNT.subscribe(false);
    private boolean doLogging = false;


    // Drivetrain logging/telemetry objects
    private final DoubleArrayLogEntry chassisPoseLog = new DoubleArrayLogEntry(m_log0, "DriveState/chassisPose");
    private final StructPublisher<Pose2d> chassisPoseNT = evenTable.getStructTopic("DriveState/chassisPose", Pose2d.struct).publish();

    private final DoubleArrayLogEntry chassisVelocityLog = new DoubleArrayLogEntry(m_log0, "DriveState/chassisVelocity");
    private final StructPublisher<ChassisSpeeds> chassisVelocityNT = evenTable.getStructTopic("DriveState/chassisVelocity", ChassisSpeeds.struct).publish();

    private final DoubleArrayPublisher fieldPub = evenTable.getDoubleArrayTopic("DriveState/Pose/Robot").publish();
    private final StringPublisher fieldTypePub = evenTable.getStringTopic("DriveState/Pose/.type").publish();
    private final double[] poseArray = new double[3];

    // Match time
    private final DoublePublisher matchTime = evenTable.getDoubleTopic("MatchTime").publish();

    // Who won auton?
    private final StringArrayPublisher autonWinner = evenTable.getStringArrayTopic("AutonWinner").publish();
    private final String[] autonWinnerColorNone = new String[] {"#FF0000", "#0000FF"};
    private final String[] autonWinnerColorError = new String[] {"#00FF00", "#00FF00"};
    private final String[] autonWinnerColorRed = new String[] {"#FF0000", "#000000"};
    private final String[] autonWinnerColorBlue = new String[] {"#000000", "#0000FF"};

    public Logging() {
        // Initialize the doLogging switch
        doLoggingNTPub.set(doLogging);
    }

    @Override
    public void periodic() {
        // Should we be logging? Ask the driver/programmer 
        // unless FMS is attatched. Logging must be done if it is
        if (DriverStation.isFMSAttached()) {
            doLogging = true;
            doLoggingNTPub.set(doLogging);
        } else {
            doLogging = doLoggingNTSub.get();
        }

        // Give driver match time
        matchTime.set(DriverStation.getMatchTime());
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
