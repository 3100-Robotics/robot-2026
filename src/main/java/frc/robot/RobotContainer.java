// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Break out auton stuff to a seperate file

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexer;

public class RobotContainer {
    private final CommandXboxController driverCtl = new CommandXboxController(0);
    private final CommandXboxController coDriverCtl = new CommandXboxController(1);

    // Shooter (This one is Evens favorite)
    // private Shooter shooter;

    // Indexer
    private Indexer indexer;

    // Intake
    // private IntakePivot intakePivot;
    // private IntakeRoller intakeRoller;

    // Drivetrain
    // private Drivetrain drivetrain;
    // private double MaxSpeed = 1.0 * TunerConstantsArkelon.kSpeedAt12Volts.in(MetersPerSecond); //
    // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a
    // rotation per second max angular velocity

    // // Root swerve requests
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1)
    //         .withRotationalDeadband(MaxAngularRate * 0.1)
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for
    // drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // // Vision
    // private Vision vision;

    // private Logging log = new Logging();

    // private Locator locator;

    // public Auton autoManager;

    @SuppressWarnings("unused")
    public RobotContainer() {
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

        if (Constants.enableIndexer) {
            indexer = new Indexer();
        }

        configureBindings();
    }

    private void configureBindings() {
        driverCtl
                .a()
                .onTrue(indexer.setSetpointsOfRollers(Constants.Indexer.on))
                .onFalse(indexer.setSetpointsOfRollers(Constants.Indexer.off));
    }
}
