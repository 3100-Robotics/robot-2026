// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Break out auton stuff to a seperate file


package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
    private final CommandXboxController driverCtl = new CommandXboxController(0);
    private final CommandXboxController coDriverCtl = new CommandXboxController(1);

    // Shooter (This one is Evens favorite)
    private Shooter shooter;

    // Indexer
    private Indexer indexer;

    // Intake
    private Intake intake;

    // Drivetrain
    private Drivetrain drivetrain;

    private Logging log = new Logging();

    // private Locator locator = new Locator(drivetrain::getPos);


    @SuppressWarnings("unused")
    public RobotContainer() {
        // Gets rid of a extremely minor error message only sim,
        // because it's a very (very!) worrying error on a real robot
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        // Check if any subsystems are disabled
        if (!Constants.enableShooter || 
            !Constants.enableIndexer ||
            !Constants.enableIntake ||
            !Constants.enableDrivetrain) 
        {
            // If any subsystems are disabled live tuning must be on
            // Crash the code because this is a hardcoded error in
            // a deploy of the code and must be fixed.
            if (!Constants.doLiveTuning) {
                int e = 0;
                var o = 1/e;
            } else {
                // Live tuning is enabled, dont bother configuring bindings,
                // but go ahead and make the subsytems we want
            }
        }

        if (Constants.enableShooter) {
            shooter = new Shooter();
        }

        if (Constants.enableIndexer) {
            indexer = new Indexer();
        }

        if (Constants.enableIntake) {
            intake = new Intake();
        }

        if (Constants.enableDrivetrain) {
            // drivetrain = new Drivetrain();
            // drivetrain.registerTelemetry(log::logCTREChassis);
        }

        if (!Constants.doLiveTuning) {
            // Only bother configuring bindings if live tuning off
            configureBindings();
        }
    }

    private void configureBindings() {
        // Intake bindings
        // driverCtl.x().whileTrue(intake.stow());
        // driverCtl.y().whileTrue(intake.runAtSpeed(RPM.of(3000)));


        // Idle bindings
        driverCtl.povUp().or(coDriverCtl.povUp()).onTrue(
            Commands.parallel(
                intake.deploy(),
                intake.stop(),
                indexer.idle(),
                shooter.idle()
            )
        );

        //coDriverCtl.povDown().onTrue(
            // shooter.stopFlywheels
        //);

        // driverCtl.a().whileTrue(); // Autoalign

        // Intake
        coDriverCtl.rightBumper().onTrue(
            Commands.parallel(
                intake.deploy(),
                intake.runAtSpeed(RPM.of(6000))
            )
        );
        coDriverCtl.leftBumper().onTrue(intake.stow());

        // Adj hood down/up
        coDriverCtl.povLeft().onTrue(
            Commands.runOnce(
                () -> shooter.setHoodAngleSetpoint(
                    Degrees.of(shooter.getHoodAngleSetpoint().in(Degrees) - 2)
                )
            )
        );

        coDriverCtl.povRight().onTrue(
            Commands.runOnce(
                () -> shooter.setHoodAngleSetpoint(
                    Degrees.of(shooter.getHoodAngleSetpoint().in(Degrees) + 2)
                )
            )
        );


        // Adj flywheel slower/faster
        coDriverCtl.leftBumper().onTrue(
            Commands.runOnce(
                () -> shooter.setSpeedSetpoint(
                    RPM.of(shooter.getSpeedSetpoint().in(RPM) - 100)
                )
            )
        );
        coDriverCtl.rightBumper().onTrue(
            Commands.runOnce(
                () -> shooter.setSpeedSetpoint(
                    RPM.of(shooter.getSpeedSetpoint().in(RPM) + 100)
                )
            )
        );
    }

    public Command getAutonomousCommand() {
       return Commands.print("No autonomous command configured");
    }
}














// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import static edu.wpi.first.units.Units.*;

// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Drivetrain;

// public class RobotContainer {
//     private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
//     private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

//     /* Setting up bindings for necessary control of the swerve drive platform */
//     private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
//             .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
//             .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
//     private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
//     private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

//     private final Telemetry logger = new Telemetry(MaxSpeed);

//     private final CommandXboxController joystick = new CommandXboxController(0);

//     public final Drivetrain drivetrain = TunerConstants.createDrivetrain();

    

//     private Vision vision;



//     public RobotContainer() {
//         if (RobotBase.isSimulation()) {
//             DriverStation.silenceJoystickConnectionWarning(true);
//         }
//         vision = new Vision(RobotBase.isSimulation(), drivetrain::addVisionMeasurement, () -> drivetrain.getState().Pose);
//         configureBindings();
//     }

//     private void configureBindings() {
//         // Note that X is defined as forward according to WPILib convention,
//         // and Y is defined as to the left according to WPILib convention.
//         drivetrain.setDefaultCommand(
//             // Drivetrain will execute this command periodically
//             drivetrain.applyRequest(() ->
//                 drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
//                     .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
//                     .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
//             )
//         );

//         // Idle while the robot is disabled. This ensures the configured
//         // neutral mode is applied to the drive motors while disabled.
//         final var idle = new SwerveRequest.Idle();
//         RobotModeTriggers.disabled().whileTrue(
//             drivetrain.applyRequest(() -> idle).ignoringDisable(true)
//         );

//         joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
//         joystick.b().whileTrue(drivetrain.applyRequest(() ->
//             point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
//         ));

//         // Run SysId routines when holding back/start and X/Y.
//         // Note that each routine should be run exactly once in a single log.

//         // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
//         // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
//         // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
//         // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

//         // Reset the field-centric heading on left bumper press.
//         joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

//         drivetrain.registerTelemetry(logger::telemeterize);
//     }

//     public Command getAutonomousCommand() {
//         // Simple drive forward auton
//         final var idle = new SwerveRequest.Idle();
//         return Commands.sequence(
//             // Reset our field centric heading to match the robot
//             // facing away from our alliance station wall (0 deg).
//             drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
//             // Then slowly drive forward (away from us) for 5 seconds.
//             drivetrain.applyRequest(() ->
//                 drive.withVelocityX(0.5)
//                     .withVelocityY(0)
//                     .withRotationalRate(0)
//             )
//             .withTimeout(5.0),
//             // Finally idle for the rest of auton
//             drivetrain.applyRequest(() -> idle)
//         );
//     }
// }
