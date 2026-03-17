package frc.robot.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Locator;
import frc.robot.RobotContainer;
import frc.robot.Vision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

@SuppressWarnings("unused")
public class Auton {
    private AutoChooser autoChooser = new AutoChooser();
    private AutoFactory autoFactory;

    // Subsystem refs
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Indexer indexer;
    private Intake intake;
    private Vision vision;
    private RobotContainer rcontainer;

    public Trigger astop = new Trigger(() -> SmartDashboard.getBoolean("astop",
                        false
                    ));

    public Auton() {
        this.rcontainer = rcontainer;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.vision = vision;

        autoFactory = new AutoFactory(
            this.drivetrain::getPos,
            this.drivetrain::resetPose,
            this.drivetrain::followTrajectory,
            true,
            this.drivetrain
        );

        autoChooser.addRoutine("Left2", this::left2);
        autoChooser.addRoutine("Outpost Only", this::outpostOnly);
        autoChooser.addRoutine("Development Outpost Only", this::developmentOutpostOnly);
        autoChooser.addRoutine("Cross Bump", this::crossBump);
        autoChooser.addRoutine("Development Cross Bump", this::crossBumpDevelopment);
        autoChooser.addRoutine("Cross Bump Development Two", this::crossBumpDevelopmentTwo);

        SmartDashboard.putData("Auton Selector", autoChooser);
        SmartDashboard.putBoolean("astop", false);
        RobotModeTriggers.autonomous()
            .whileTrue(autoChooser.selectedCommandScheduler()
                .unless(astop)
        );

        RobotModeTriggers.teleop()
            .or(astop)
            .onTrue(
                Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll())
                .alongWith(Commands.runOnce(() -> vision.usePose = true))
            );
    }

    public AutoRoutine left2() {
        var routine = autoFactory.newRoutine("Left 2");
        AutoTrajectory leftStart = routine.trajectory("leftStart");
        routine.active().onTrue(
            Commands.parallel(
                intake.halfway(),

                Commands.sequence(
                    Commands.runOnce(() -> SmartDashboard.putString("astage", "zeroth")),
                    autoFactory.resetOdometry("leftStart"),
                    leftStart.cmd(),
                    Commands.runOnce(() -> SmartDashboard.putString("astage", "first")),

                    Commands.parallel(
                        drivetrain.pointAtPose(() -> Locator.getInstance().hubPose),
                        Commands.race(
                            rcontainer.shoot(),
                            Commands.waitSeconds(10)
                        )
                    ),
                    Commands.parallel(indexer.idle(),
                        shooter.idle()),
                    Commands.runOnce(() -> intake.deployed = true)
                )

            )
        );
        return routine;
    }

    public AutoRoutine outpostOnly() {
        var routine = autoFactory.newRoutine("Outpost only");
        AutoTrajectory rightToOutpost = routine.trajectory("rightToOutpost_part1");

        routine.active().onTrue(
            Commands.parallel(
                Commands.runOnce(() -> drivetrain.speedMultiplier = 0.2),

                Commands.sequence(
                    Commands.runOnce(() -> intake.deployed = true),
                    Commands.runOnce(() -> SmartDashboard.putString("astage", "zeroth")),
                    autoFactory.resetOdometry("rightToOutpost_part1"),
                    // rightToOutpost.cmd(),
                    Commands.runOnce(() -> SmartDashboard.putString("astage", "first")),
                    drivetrain.goToPoseCommand(() -> rightToOutpost.getFinalPose().get())
                        .until(() -> drivetrain.isAtPoseSetpoint(false)),

                    Commands.run(() -> drivetrain.setControl(
                        new SwerveRequest.RobotCentric()
                            .withVelocityX(MetersPerSecond.of(0.15))
                            .withVelocityY(0)
                            .withRotationalRate(0)
                    ))
                    .finallyDo(() -> drivetrain.setControl(new SwerveRequest.Idle()))
                    .withTimeout(2),

                    Commands.runOnce(() -> SmartDashboard.putString("astage", "second")),
                    Commands.waitSeconds(3),
                    Commands.runOnce(() -> SmartDashboard.putString("astage", "third")),

                    Commands.runOnce(() -> intake.deployed = false),

                    Commands.race(
                        drivetrain.goToPoseCommandStatic(() -> Locator.getInstance().extentionPose),
                        Commands.waitSeconds(2.5)
                    ),
                    Commands.parallel(
                        intake.runAtSpeed(RPM.of(4000)),
                        Commands.race(
                            rcontainer.shoot(),
                            Commands.waitSeconds(10)
                        ).andThen(rcontainer.idleAll())
                    ),
                    Commands.parallel(indexer.idle(),
                        shooter.idle(),
                        shooter.idleFlywheels()),
                    Commands.runOnce(() -> intake.deployed = true)
                )

            )
        );
        return routine;
    }

    public AutoRoutine crossBump() {
        var routine = autoFactory.newRoutine("crossBump");
        
        var part1 = routine.trajectory("crossBump_part1");
        var part2 = routine.trajectory("crossBump_part2");
        var part25 = routine.trajectory("crossBump_part25");
        var part3 = routine.trajectory("crossBump_part3");
        var part4 = routine.trajectory("crossBump_part4");
        

        routine.active().onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.runOnce(() -> drivetrain.speedMultiplier = 0.4),
                Commands.runOnce(() -> intake.deployed = true),
                drivetrain.goToPoseCommand(() -> part1.getInitialPose().get())
                    .withTimeout(0.2),
                drivetrain.goToPoseCommand(() -> part2.getInitialPose().get())
                    .withTimeout(2),
                drivetrain.goToPoseCommand(() -> part2.getFinalPose().get())
                    .withTimeout(2),
                Commands.runOnce(() -> vision.usePose = false),
                drivetrain.goToPoseCommand(() -> part25.getInitialPose().get())
                    .withTimeout(1.5),
                // Start intake here
                Commands.runOnce(() -> drivetrain.speedMultiplier = 0.75),
                drivetrain.goToPoseCommand(() -> part3.getInitialPose().get())
                    .alongWith(intake.runAtSpeed(RPM.of(3000)))
                    .withTimeout(1.5),
                intake.stop().withTimeout(0),
                Commands.runOnce(() -> drivetrain.speedMultiplier = 1),
                // End here
                drivetrain.goToPoseCommand(() -> part3.getFinalPose().get())
                    .withTimeout(1.3),
                Commands.run(() -> drivetrain.setControl(
                    new SwerveRequest.RobotCentric()
                        .withVelocityX(MetersPerSecond.of(-2))
                        .withVelocityY(0)
                        .withRotationalRate(0)
                )).withTimeout(2)
                .finallyDo(() -> drivetrain.setControl(new SwerveRequest.Idle())),
                Commands.runOnce(() -> vision.usePose = true),
                // drivetrain.goToPoseCommand(() -> part4.getInitialPose().get())
                //     .withTimeout(3),
                Commands.race(
                    drivetrain.goToPoseCommandStatic(() -> Locator.getInstance().extentionPose),
                    Commands.waitSeconds(2.5)
                ),
                Commands.runOnce(() -> intake.deployed = false),

                Commands.parallel(
                    intake.runAtSpeed(RPM.of(4000)),
                    Commands.race(
                        rcontainer.shoot(),
                        Commands.waitSeconds(10)
                    ).andThen(rcontainer.idleAll())
                ),
                Commands.parallel(indexer.idle(),
                    shooter.idleFlywheels())
            )
        );

        return routine;
    }
}