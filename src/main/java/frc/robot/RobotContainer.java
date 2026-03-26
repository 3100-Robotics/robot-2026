// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.Auton;
import frc.robot.generated.TunerConstantsArkelon0306Duluth;
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
    private double MaxSpeed = 1.0 * TunerConstantsArkelon0306Duluth.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt pointWheelsRobotForward = new SwerveRequest.PointWheelsAt()
        .withModuleDirection(Rotation2d.kZero);

    // Vision
    private Vision vision;

    private Logging log = new Logging();

    private Locator locator;

    public Auton autoManager;

    private PowerDistribution pdh = new PowerDistribution(60, ModuleType.kRev);
    private List<DoubleLogEntry> currentLogs = new ArrayList<DoubleLogEntry>();
    private boolean nologging = false;

    @SuppressWarnings("unused")
    public RobotContainer(Robot robot) {
        SmartDashboard.putBoolean("isLggingCurrent", true);
        try {
            for (int i = 0; i < pdh.getAllCurrents().length; i++) {
                currentLogs.add(i,
                    new DoubleLogEntry(
                        log.m_log0,
                        String.format("/pdhCurrents/_%d", i)
                    )
                );
            }
            robot.addPeriodic(this::logCurrents, 0.04);
        } catch (Exception e) {
            SmartDashboard.putBoolean("isLggingCurrent", false);
            nologging = true;
        }
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
            drivetrain = TunerConstantsArkelon0306Duluth.createDrivetrain();
            drivetrain.registerTelemetry(log::logCTREChassis);

            vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getPos);
            locator = new Locator(drivetrain::getPos);
            autoManager = new Auton(drivetrain, shooter, indexer, intake, this, vision);
        }

        if (!Constants.doLiveTuning) {
            // Only bother configuring bindings if live tuning off
            configureBindings();
        }
    }

    public void logCurrents() {
        SmartDashboard.putNumber("testPDHCurrent", currentLogs.get(4).getLastValue());
        if (nologging) {
            return;
        }

        try {
            var allCurrents = pdh.getAllCurrents();
            for (int i = 0; i < allCurrents.length; i++) {
                currentLogs.get(i).append(allCurrents[i]);
            }
        } catch (Exception e) {
            nologging = true;
            SmartDashboard.putBoolean("isLggingCurrent", false);
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

    public Command shootDialed() {
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
                Commands.waitSeconds(0.08),
                Commands.waitUntil(shooter.flywheelsAtRPMAcceleration),
                indexer.run()
            )
        );
    }

    public Command idleAll() {
        return Commands.parallel(indexer.idle(), shooter.idleFlywheels());
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
        driverCtl.b().or(driverCtl.povDown()).whileTrue(
            drivetrain.applyRequest(() -> brake)
        );

        driverCtl.rightBumper().onTrue(Commands.runOnce(() -> {
            if (vision.usePose == true) {
                vision.usePose = false;
            } else {
                vision.usePose = true;
            }
        }));

        // Autoalign to hub
        driverCtl.a().whileTrue(
            drivetrain.pointAtPose(() -> locator.hubPose)
        );

        // Spin up flywheels
        driverCtl.x().onTrue(
            shooter.toggleKeepSpunUp()
        );

        shooter.keepSpunUpTrigger.whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> {
                        var targets = shooter.calculateFireAngleAndSpeed();
                        shooter.setHoodAngleSetpoint(targets.getFirst());
                        shooter.setSpeedSetpoint(targets.getSecond());
                    }
                ),
                shooter.goToCurrentAngle(),
                shooter.runFlywheelsToCurrent()
            )
        ).whileFalse(
            Commands.parallel(
                indexer.idle(),
                shooter.idle()
            )
        );

        // driverCtl.y().whileTrue(drivetrain.goToPoseCommand(() -> locator.extentionPose));
        driverCtl.y().whileTrue(
            drivetrain.applyRequest(() -> pointWheelsRobotForward)
        );

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

        // Reverse indexer rollers
        coDriverCtl.povLeft().whileTrue(
            indexer.runRev()
        ).whileFalse(indexer.idle());

        // Toggle deployed or not
        coDriverCtl.b().or(coDriverCtl.leftBumper()).onTrue(
            intake.toggleDeploy()
        );



        /// Shooter
        coDriverCtl.a().whileTrue(shootDialed()).whileFalse(
            Commands.parallel(
                indexer.idle(),
                shooter.idle()
            )
        );
        // coDriverCtl.a().whileTrue(
        //     Commands.parallel(
        //         Commands.run(
        //             () -> {
        //                 var targets = shooter.calculateFireAngleAndSpeed();
        //                 shooter.setHoodAngleSetpoint(targets.getFirst());
        //                 shooter.setSpeedSetpoint(targets.getSecond());
        //             }
        //         ),
        //         shooter.goToCurrentAngle(),
        //         shooter.runFlywheelsToCurrent(),
        //         Commands.sequence(
        //             Commands.waitSeconds(1.1),
        //             indexer.run()
        //         )
        //     )
        // ).whileFalse(
        //     Commands.parallel(
        //         indexer.idle(),
        //         shooter.idle()
        //     )
        // );

        coDriverCtl.y().whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> {
                        shooter.setHoodAngleSetpoint(Degrees.of(17));
                        shooter.setSpeedSetpoint(RPM.of(2600));
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
