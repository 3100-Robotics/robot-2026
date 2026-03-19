// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.Auton;
import frc.robot.generated.TunerConstantsArkelon0306Duluth;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.ShooterFlywheel;
import frc.robot.subsystems.shooter.ShooterHood;

public class RobotContainer {

    private static final RobotContainer instance = new RobotContainer();
    public static RobotContainer getInstance() {
        return instance;
    }


    private final CommandXboxController driverCtl = new CommandXboxController(0);
    private final CommandXboxController coDriverCtl = new CommandXboxController(1);

    private final PowerDistribution pdh = new PowerDistribution(60, ModuleType.kRev);

    // Shooter (This one is Evens favorite)
    public ShooterHood shooterHood;
    public ShooterFlywheel shooterFlywheelL;
    public ShooterFlywheel shooterFlywheelR;

    // Indexer
    public Indexer indexer;

    // Intake
    public IntakePivot intakePivot;
    public IntakeRoller intakeRoller;

    // Drivetrain
    public Drivetrain drivetrain;
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
    public Vision vision;

    private Logging log = new Logging();

    public Locator locator;

    public Auton autoManager;

    private RobotContainer() {
        // Gets rid of a extremely minor error message only sim,
        // because it's a very (very!) worrying error on a real robot
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        intakePivot = new IntakePivot();
        intakeRoller = new IntakeRoller();

        indexer = new Indexer();

        // if (Constants.enableDrivetrain) {
        //     drivetrain = TunerConstantsArkelon0306Duluth.createDrivetrain();
        //     drivetrain.registerTelemetry(log::logCTREChassis);

        //     vision = new Vision(drivetrain::addVisionMeasurement, drivetrain::getPos);
        //     locator = new Locator(drivetrain::getPos);
        //     autoManager = new Auton();
        // }

        configureBindings();
    }

    public double getPDHCurrentFromChannel(int channel) {
        return pdh.getCurrent(channel);
    }

    private void configureBindings() {
        // driverCtl.a().onTrue(intakePivot.toggle());
        // driverCtl.b().onTrue(intakeRoller.on()).onFalse(intakeRoller.off());

        driverCtl.a().onTrue(indexer.on()).onFalse(indexer.off());
        // driverCtl.b().onTrue(intakeRoller.on()).onFalse(intakeRoller.off());
    }
}
