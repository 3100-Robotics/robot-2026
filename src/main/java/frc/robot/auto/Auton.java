package frc.robot.auto;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.util.WPILibVersion;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Locator;
import frc.robot.RobotContainer;
import frc.robot.Vision;
import frc.robot.commands.DriveRobotOriented;
import frc.robot.math.Direction;
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

    public Auton(
        Drivetrain drivetrain,
        Shooter shooter,
        Indexer indexer,
        Intake intake,
        RobotContainer rcontainer,
        Vision vision
    ) {
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

        autoChooser.addRoutine("Right DEVELOPMENT FLAG Cross Bump", this::crossBumpDevelopment);

        autoChooser.addRoutine("Left BiblicalGree", () -> biblicalGreedAuton(Direction.Left));
        autoChooser.addRoutine("Right BiblicalGreed", () -> biblicalGreedAuton(Direction.Right));
        autoChooser.addRoutine("Right Flagship Cross Bump", this::flagshipCrossBump);

        autoChooser.addRoutine("Right DEVELOPMENT Cross Bump", () -> sideCrossBump(Direction.Right));
        autoChooser.addRoutine("Left DEVELOPMENT Cross Bump", () -> sideCrossBump(Direction.Left));

        autoChooser.addRoutine("Right Outpos", this::collectOutpostAndScore);

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

    public AutoRoutine scorePreloadOnly() {
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

    public AutoRoutine collectOutpostAndScore() {
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


    public AutoRoutine sideCrossBump(Direction side) {
        var routine = autoFactory.newRoutine("crossBump");
        
        var part1 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBump_part1"));
        var part2 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBump_part2"));
        var part25 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBump_part25"));
        var part3 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBump_part3"));
        var part4 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBump_part4"));
        

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

    public AutoRoutine flagshipCrossBump() {
        var routine = autoFactory.newRoutine("crossBump");
        
        var part1 = routine.trajectory("crossBump_part1");
        var part2 = routine.trajectory("crossBump_part2");
        var part25 = routine.trajectory("crossBump_part25");
        var part3 = routine.trajectory("crossBump_part3");
        var part4 = routine.trajectory("crossBump_part4");
        

        routine.active().onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.runOnce(() -> drivetrain.speedMultiplier = 1),
                Commands.runOnce(() -> intake.deployed = false),
                drivetrain.goToPoseCommand(() -> part1.getInitialPose().get())
                    .withTimeout(2),
                drivetrain.goToPoseCommand(() -> part2.getInitialPose().get())
                    .withTimeout(2),
                drivetrain.goToPoseCommand(() -> part2.getFinalPose().get())
                    .withTimeout(2),
                Commands.runOnce(() -> vision.usePose = false),
                drivetrain.goToPoseCommand(() -> part25.getInitialPose().get())
                    .withTimeout(2),
                // Start intake here
                Commands.runOnce(() -> intake.deployed = true),
                Commands.runOnce(() -> drivetrain.speedMultiplier = 0.75),
                drivetrain.goToPoseCommand(() -> part3.getInitialPose().get())
                    .alongWith(intake.runAtSpeed(RPM.of(3000)))
                    .until(() -> drivetrain.isAtPoseSetpoint(false)),
                intake.stop().withTimeout(0),
                Commands.runOnce(() -> intake.deployed = false),
                Commands.runOnce(() -> drivetrain.speedMultiplier = 1),
                // End here
                drivetrain.goToPoseCommand(() -> part3.getFinalPose().get())
                    .withTimeout(1.3),
                Commands.run(() -> drivetrain.setControl(
                    new SwerveRequest.RobotCentric()
                        .withVelocityX(MetersPerSecond.of(-2.5))
                        .withVelocityY(MetersPerSecond.of(2.5))
                        .withRotationalRate(0)
                )).withTimeout(1)
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


    public AutoRoutine crossBumpDevelopment() {
        var routine = autoFactory.newRoutine("crossBump");
        
        var part1 = routine.trajectory("crossBump_part1");
        var part2 = routine.trajectory("crossBump_part2");
        var part25 = routine.trajectory("crossBump_part25");
        var part3 = routine.trajectory("crossBump_part3");
        var part4 = routine.trajectory("crossBump_part4");

        var dpart1 = routine.trajectory("DEVcrossBump_part1");
        var dpart25 = routine.trajectory("DEVcrossBump_part25");
        var dpart3 = routine.trajectory("DEVcrossBump_part3");

        var big = routine.trajectory("crossBumpBig");
        var bigSweep2 = routine.trajectory("crossBumpSweep2");

        routine.active().onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.runOnce(() -> drivetrain.speedMultiplier = 1),
                Commands.runOnce(() -> intake.deployed = true),

                // drivetrain.goToPoseCommand(() -> part1.getInitialPose().get())
                //     .withTimeout(0.2), // Go to the total initial pose
                // drivetrain.goToPoseCommand(() -> part2.getInitialPose().get())
                //     .withTimeout(2), // Go to the other side of the bump

                new DriveRobotOriented(drivetrain, false, MetersPerSecond.of(-2.5), MetersPerSecond.of(-2.5), RPM.of(0))
                    .withTimeout(1.5), // Cross bump

                drivetrain.goToPoseCommand(() -> big.getInitialPose().get())
                    .until(() -> drivetrain.isAtPoseSetpoint(false)),
                // big.cmd(), // Line up with fuel

                intake.runAtSpeed(RPM.of(4000)).withTimeout(0), // Run intake
                new DriveRobotOriented(drivetrain, true, MetersPerSecond.of(2), MetersPerSecond.of(0), RPM.of(0))
                    .withTimeout(1.5), // Sweep balls
                intake.stop().withTimeout(0), // Kill intake


                // Max speed (on positions)
                Commands.runOnce(() -> drivetrain.speedMultiplier = 1),
                

                drivetrain.goToPoseCommand(() -> dpart3.getFinalPose().get())
                    .withTimeout(1.3),

                new DriveRobotOriented(drivetrain, MetersPerSecond.of(-2), MetersPerSecond.of(2), RPM.of(0))
                    .withTimeout(1),

                Commands.runOnce(() -> vision.usePose = true),
                // drivetrain.goToPoseCommand(() -> part4.getInitialPose().get())
                //     .withTimeout(3),

                Commands.race(
                    drivetrain.goToPoseCommandStatic(() -> Locator.getInstance().extentionPose),
                    Commands.waitSeconds(2.5)
                ),
                // drivetrain.pointAtPose(() -> Locator.getInstance().hubPose)
                //     .until(() -> drivetrain.isAtPoseSetpoint(true)),

                Commands.runOnce(() -> intake.deployed = false),

                Commands.parallel(
                    Commands.parallel(
                        intake.runAtSpeed(RPM.of(4000)),
                        rcontainer.shootDialed()
                    ).withTimeout(5).andThen(
                        Commands.sequence(
                            Commands.runOnce(() -> shooter.setSpeedSetpoint(RPM.of(0))),
                            Commands.parallel(intake.stop(), indexer.idle(),
                                shooter.runFlywheelsToCurrent()).withTimeout(0)
                        )
                    ),
                    Commands.waitSeconds(4.7)
                        .andThen(
                            // Line up for crossing bump
                            drivetrain.goToPoseCommand(() -> dpart1.getInitialPose().get())
                                .until(() -> drivetrain.isAtPoseSetpoint(false))
                        )
                ),
                
                // Line up for crossing bump
                // drivetrain.goToPoseCommand(() -> dpart1.getInitialPose().get())
                //     .until(() -> drivetrain.isAtPoseSetpoint(false)),
                
                new DriveRobotOriented(drivetrain, false, MetersPerSecond.of(-2.7), MetersPerSecond.of(-2.7), RPM.of(0))
                    .withTimeout(0.8), // Cross bump

                drivetrain.goToPoseCommand(() -> bigSweep2.getInitialPose().get())
                    .withTimeout(0.7),

                new DriveRobotOriented(drivetrain, false, MetersPerSecond.of(3), MetersPerSecond.of(0), RPM.of(0))
                    .withTimeout(1), // Get balls

                new DriveRobotOriented(drivetrain, false, MetersPerSecond.of(-3), MetersPerSecond.of(0), RPM.of(0))
                    .withTimeout(1) // Come back
            )
        );

        return routine;
    }

    public AutoRoutine biblicalGreedAuton(Direction side) {
        var routine = autoFactory.newRoutine("biblicalGreedAuton"+side.toString());
        
        var part1 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBump_part1"));
        var part2 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBump_part2"));
        var part25 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBump_part25"));
        var part3 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBump_part3"));
        var part4 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBump_part4"));

        var dpart1 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("DEVcrossBump_part1"));
        var dpart25 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("DEVcrossBump_part25"));
        var dpart3 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("DEVcrossBump_part3"));

        var big = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBumpBig"));
        var bigSweep2 = FlipTrajectory.flipConditional(side, routine, routine.trajectory("crossBumpSweep2"));

        var biblical = FlipTrajectory.flipConditional(side, routine, routine.trajectory("biblical"));

        routine.active().onTrue(
            Commands.sequence(
                part1.resetOdometry(),
                Commands.runOnce(() -> drivetrain.speedMultiplier = 1),
                Commands.runOnce(() -> intake.deployed = true),

                new DriveRobotOriented(drivetrain,
                    false,
                    MetersPerSecond.of(-2.5),
                    MetersPerSecond.of(-2.5),
                    RPM.of(0)
                ).withSide(side).withTimeout(0.7), // Cross bump

                drivetrain.goToPoseCommand(() -> big.getInitialPose().get())
                    .until(() -> drivetrain.isAtPoseSetpoint(false)),
                big.cmd(), // Line up with fuel

                drivetrain.goToPoseCommand(() -> biblical.getInitialPose().get())
                    .until(() -> drivetrain.isAtPoseSetpoint(false)),
                drivetrain.goToPoseCommand(() -> biblical.getFinalPose().get())
                    .until(() -> drivetrain.isAtPoseSetpoint(false)),


                // Max speed (on positions)
                Commands.runOnce(() -> drivetrain.speedMultiplier = 1),
                

                drivetrain.goToPoseCommand(() -> dpart3.getFinalPose().get())
                    .withTimeout(1.3),

                new DriveRobotOriented(drivetrain, true, MetersPerSecond.of(-2), MetersPerSecond.of(2), RPM.of(0))
                    .withSide(side)
                    .withTimeout(1),

                Commands.runOnce(() -> vision.usePose = true),
                // drivetrain.goToPoseCommand(() -> part4.getInitialPose().get())
                //     .withTimeout(3),

                // Commands.race(
                //     drivetrain.goToPoseCommandStatic(() -> Locator.getInstance().extentionPose),
                //     Commands.waitSeconds(2.5)
                // ),
                drivetrain.pointAtPose(() -> Locator.getInstance().hubPose)
                    .until(() -> drivetrain.isAtPoseSetpoint(true))
                    .andThen(new DriveRobotOriented(drivetrain, 
                        MetersPerSecond.of(0),
                        MetersPerSecond.of(0),
                        RotationsPerSecond.of(0)
                    ).withTimeout(0)),

                Commands.runOnce(() -> intake.deployed = false),

                Commands.parallel(
                    Commands.parallel(
                        intake.runAtSpeed(RPM.of(4000)),
                        rcontainer.shootDialed()
                    ).withTimeout(5).andThen(
                        Commands.sequence(
                            Commands.runOnce(() -> shooter.setSpeedSetpoint(RPM.of(0))),
                            Commands.parallel(intake.stop(), indexer.idle(),
                                shooter.runFlywheelsToCurrent()).withTimeout(0)
                        )
                    ),
                    Commands.waitSeconds(4.7)
                        .andThen(
                            // Line up for crossing bump
                            drivetrain.goToPoseCommand(() -> dpart1.getInitialPose().get())
                                .until(() -> drivetrain.isAtPoseSetpoint(false))
                        )
                ),
                
                // Line up for crossing bump
                // drivetrain.goToPoseCommand(() -> dpart1.getInitialPose().get())
                //     .until(() -> drivetrain.isAtPoseSetpoint(false)),
                
                new DriveRobotOriented(drivetrain, false, MetersPerSecond.of(-2.7), MetersPerSecond.of(-2.7), RPM.of(0))
                    .withSide(side)
                    .withTimeout(0.8), // Cross bump

                drivetrain.goToPoseCommand(() -> bigSweep2.getInitialPose().get())
                    .withTimeout(0.7),

                new DriveRobotOriented(drivetrain, false, MetersPerSecond.of(3), MetersPerSecond.of(0), RPM.of(0))
                    .withTimeout(1), // Get balls

                new DriveRobotOriented(drivetrain, false, MetersPerSecond.of(-3), MetersPerSecond.of(0), RPM.of(0))
                    .withTimeout(1) // Come back
            )
        );

        return routine;
    }
}