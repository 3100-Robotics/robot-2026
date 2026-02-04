// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Break out auton stuff to a seperate file

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstantsV1Protobot;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.util.datalog.StringLogEntry;

public class RobotContainer {
    private final DataLog m_log0 = DataLogManager.getLog();
    private final StringLogEntry testStringLog = new StringLogEntry(m_log0, "/root/teststring");

    private final CommandXboxController driverCtl = new CommandXboxController(0);
    private final CommandXboxController coDriverCtl = new CommandXboxController(1);
    
    // Drivetrain
    private final double m_maxSpeed = TunerConstantsV1Protobot.kSpeedAt12Volts.in(MetersPerSecond)/4; // Get real max speed
    private final double m_maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric m_driveFieldCentric = new SwerveRequest.FieldCentric()
        .withDeadband(m_maxSpeed * 0.09)
        .withRotationalDeadband(m_maxAngularRate * 0.09) // Experiment with best values for these but they worked on Hawksbill
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage); 
    private Drivetrain drivetrain = TunerConstantsV1Protobot.createDrivetrain();


    // Other Subsystems
    // private final Intake intake = new Intake();





    // private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // /* Setting up bindings for necessary control of the swerve drive platform */
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    // private final CommandXboxController joystick = new CommandXboxController(0);



    private Logging log = new Logging();

    private Locator locator = new Locator(drivetrain::getPos);

    public final AutoChooser autoSelector = new AutoChooser();

    private final AutoFactory autoFactory;


    public RobotContainer() {
        DriverStation.startDataLog(m_log0);

        autoFactory = new AutoFactory(
            drivetrain::getPos, // A function that returns the current robot pose
            drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
            drivetrain::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drivetrain // The drive subsystem
        );

        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        configureBindings();

        drivetrain.registerTelemetry(log::logCTREChassis);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
                () -> m_driveFieldCentric
                    .withVelocityX(driverCtl.getLeftX())
                    .withVelocityY(driverCtl.getLeftY())
                    .withRotationalRate(driverCtl.getRightX())
        ));

        driverCtl.a().whileTrue(drivetrain.positionDrive(() -> locator.extentionPose));
    }

    public Command getAutonomousCommand() {
       return Commands.print("No autonomous command configured");
    }

    public AutoRoutine leave() {
        AutoRoutine routine = autoFactory.newRoutine("leave");

        AutoTrajectory leaveTraj = routine.trajectory("leave");
        leaveTraj.getInitialPose();

        routine.active().onTrue(
                Commands.sequence(
                        leaveTraj.resetOdometry(),
                        leaveTraj.cmd()
                )
        );

        return routine;
    }
}
