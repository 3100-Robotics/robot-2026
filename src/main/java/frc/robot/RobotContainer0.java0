// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Break out auton stuff to a seperate file

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Intake;
import frc.robot.generated.TunerConstantsArkelon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer0 {
    private final CommandXboxController driverCtl = new CommandXboxController(0);
    private final CommandXboxController coDriverCtl = new CommandXboxController(1);

    // Shooter (This one is Evens favorite)
    private Shooter shooter;

    // Indexer
    private Indexer indexer;

    // Intake
    private IntakePivot intakePivot;
    private IntakeRoller intakeRoller;

    // Drivetrain
    private Drivetrain drivetrain;
    private double MaxSpeed =
            1.0
                    * TunerConstantsArkelon.kSpeedAt12Volts.in(
                            MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75)
                    .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.1)
                    .withDriveRequestType(
                            DriveRequestType
                                    .OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // Vision
    private Vision vision;

    private Logging log = new Logging();

    private Locator locator;

    // public Auton autoManager;

    @SuppressWarnings("unused")
    public RobotContainer0() {
        // Gets rid of a extremely minor error message only sim,
        // because it's a very (very!) worrying error on a real robot
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        // Check if any subsystems are disabled
        if (!Constants.enableShooter
                || !Constants.enableIndexer
                || !Constants.enableIntake
                || !Constants.enableDrivetrain) {
            // If any subsystems are disabled live tuning must be on
            // Crash the code because this is a hardcoded error in
            // a deploy of the code and must be fixed.
            if (!Constants.doLiveTuning) {
                int e = 0;
                var o = 1 / e;
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
            intakeRoller = new IntakeRoller();
            intakePivot = new IntakePivot();
        }

        if (Constants.enableDrivetrain) {
            drivetrain = TunerConstantsArkelon.createDrivetrain();
            drivetrain.registerTelemetry(log::logCTREChassis);

            vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getPos);
            locator = new Locator(drivetrain::getPos);
            // autoManager = new Auton(drivetrain, shooter, indexer, intake, this);
        }

        if (!Constants.doLiveTuning) {
            // Only bother configuring bindings if live tuning off
            configureBindings();
            // configShooterBinding();
        }
    }

    // public Command shoot() {
    //     return Commands.parallel(
    //         Commands.run(
    //             () -> {
    //                 var targets = shooter.calculateFireAngleAndSpeed();
    //                 shooter.setHoodAngleSetpoint(targets.getFirst());
    //                 shooter.setSpeedSetpoint(targets.getSecond());
    //                 // shooter.angleProvider = () -> Degrees.of(20);
    //                 // shooter.speedProvider = () -> RPM.of(4000);
    //                 // shooter.setHoodAngleSetpoint(Degrees.of(20));
    //                 // shooter.setSpeedSetpoint(RPM.of(4000));
    //             }
    //         ),
    //         shooter.goToCurrentAngle(),
    //         shooter.runFlywheelsToCurrent(),
    //         Commands.sequence(
    //             // Commands.waitUntil(shooter.flywheelsAtRPM),
    //             Commands.waitSeconds(1.1),
    //             indexer.run()
    //         )
    //     );
    // }

    private void configureBindings() {
        // Drivetrain
        if (Robot.isReal()) {
            drivetrain.setDefaultCommand(
                    // Drivetrain will execute this command periodically
                    drivetrain.applyRequest(
                            () ->
                                    drive.withVelocityX(
                                                    -driverCtl.getLeftY()
                                                            * MaxSpeed
                                                            * driverCtl.getRightTriggerAxis())
                                            .withVelocityY(
                                                    -driverCtl.getLeftX()
                                                            * MaxSpeed
                                                            * driverCtl.getRightTriggerAxis())
                                            .withRotationalRate(
                                                    -driverCtl.getRightX() * MaxAngularRate)));
        } else {
            drivetrain.setDefaultCommand(
                    // Drivetrain will execute this command periodically
                    drivetrain.applyRequest(
                            () ->
                                    drive.withVelocityX(-driverCtl.getLeftY() * MaxSpeed) // *
                                            // driverCtl.getRightTriggerAxis())
                                            .withVelocityY(-driverCtl.getLeftX() * MaxSpeed) // *
                                            // driverCtl.getRightTriggerAxis())
                                            .withRotationalRate(
                                                    -driverCtl.getRightX() * MaxAngularRate)));
        }

        // Reset odometry
        driverCtl.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));
        // Lockpose!
        driverCtl.b().whileTrue(drivetrain.applyRequest(() -> brake));

        driverCtl.rightBumper().onTrue(Commands.runOnce(() -> vision.usePose = false));

        driverCtl
                .a()
                .whileTrue(
                        // Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.Idle()))
                        // .andThen(Commands.waitSeconds(0.2))
                        // .andThen(
                        drivetrain.pointAtPose(() -> locator.hubPose)
                        // )
                        ); // Autoalign to hub

        coDriverCtl.povDown().onTrue(intakePivot.setState(Intake.PivotState.FullDeploy));
        coDriverCtl.povRight().onTrue(intakePivot.setState(Intake.PivotState.Stow));
        coDriverCtl.povLeft().onTrue(intakePivot.setState(Intake.PivotState.HalfDeploy));
    }
}
