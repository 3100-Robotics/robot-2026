// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.Auton;
import frc.robot.generated.TunerConstantsArkelon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
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
    private double MaxSpeed = 1.0 * TunerConstantsArkelon.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // Vision
    private Vision vision;

    private Logging log = new Logging();

    private Locator locator;

    public Auton autoManager;

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
            drivetrain = TunerConstantsArkelon.createDrivetrain();
            drivetrain.registerTelemetry(log::logCTREChassis);

            vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getPos);
            locator = new Locator(drivetrain::getPos);
            autoManager = new Auton(drivetrain, shooter, indexer, intake, this);
        }

        if (!Constants.doLiveTuning) {
            // Only bother configuring bindings if live tuning off
            configureBindings();
        }
    }

    public Command shoot() {
        return Commands.parallel(
            Commands.run(
                () -> {
                    var targets = shooter.calculateFireAngleAndSpeed();
                    shooter.setHoodAngleSetpoint(targets.getFirst());
                    shooter.setSpeedSetpoint(targets.getSecond());
                }
            ),
            shooter.goToCurrentAngle(),
            shooter.runFlywheelsToCurrent(),
            Commands.sequence(
                Commands.waitSeconds(1.1),
                indexer.run()
            )
        );
    }

    public Command idleAll() {
        return Commands.parallel(indexer.idle());
    }

    private void configureBindings() {
        // Drivetrain
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(-driverCtl.getLeftY() * MaxSpeed * driverCtl.getRightTriggerAxis())
                    .withVelocityY(-driverCtl.getLeftX() * MaxSpeed * driverCtl.getRightTriggerAxis())
                    .withRotationalRate(-driverCtl.getRightX() * MaxAngularRate)
            )
        );

        // Reset odometry
        driverCtl.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));
        // Lockpose!
        driverCtl.b().whileTrue(
            drivetrain.applyRequest(() -> brake)
        );

        driverCtl.rightBumper().onTrue(Commands.runOnce(() -> vision.usePose = false));

        // Autoalign to hub
        driverCtl.a().whileTrue(
            drivetrain.pointAtPose(() -> locator.hubPose)
        );

        driverCtl.y().whileTrue(drivetrain.goToPoseCommand(() -> locator.extentionPose));

        // Idle bindings
        driverCtl.povUp().or(coDriverCtl.povUp()).onTrue(
            Commands.parallel(
                intake.stop(),
                indexer.idle(),
                shooter.idle()
            )
        );



        /// Intake
        coDriverCtl.x().or(coDriverCtl.rightBumper()).whileTrue(
            Commands.parallel(
                intake.runAtSpeed(RPM.of(3000))
            )
        ).whileFalse(intake.stop());

        // Reverse intake rollers
        coDriverCtl.povLeft().whileTrue(
            indexer.runRev()
        ).whileFalse(indexer.idle());


        coDriverCtl.b().or(coDriverCtl.leftBumper()).onTrue(
            Commands.runOnce(() -> {
                if (intake.deployed) {
                    intake.deployed = false;
                } else {
                    intake.deployed = true;
                }
            })
        );



        /// Shooter
        coDriverCtl.a().whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> {
                        var targets = shooter.calculateFireAngleAndSpeed();
                        shooter.setHoodAngleSetpoint(targets.getFirst());
                        shooter.setSpeedSetpoint(targets.getSecond());
                    }
                ),
                shooter.goToCurrentAngle(),
                shooter.runFlywheelsToCurrent(),
                Commands.sequence(
                    Commands.waitSeconds(1.1),
                    indexer.run()
                )
            )
        ).whileFalse(
            Commands.parallel(
                indexer.idle(),
                shooter.idle()
            )
        );

        coDriverCtl.y().whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> {
                        shooter.setHoodAngleSetpoint(Degrees.of(30));
                        shooter.setSpeedSetpoint(RPM.of(1000));
                    }
                ),
                shooter.goToCurrentAngle(),
                shooter.runFlywheelsToCurrent(),
                Commands.sequence(
                    Commands.waitSeconds(1),
                    indexer.run()
                )
            )
        ).whileFalse(
            Commands.parallel(
                indexer.idle(),
                shooter.idle()
            )
        );
    }
}
