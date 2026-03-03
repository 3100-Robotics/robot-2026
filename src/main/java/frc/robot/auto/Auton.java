package frc.robot.auto;

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
    private RobotContainer rcontainer;

    public Trigger astop = new Trigger(() -> SmartDashboard.getBoolean("astop",
                        false
                    ));

    public Auton(
        Drivetrain drivetrain,
        Shooter shooter,
        Indexer indexer,
        Intake intake,
        RobotContainer rcontainer
    ) {
        this.rcontainer = rcontainer;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;

        autoFactory = new AutoFactory(
            this.drivetrain::getPos,
            this.drivetrain::resetPose,
            this.drivetrain::followTrajectory,
            true,
            this.drivetrain
        );

        // autoChooser.addRoutine("Outpose and Score", this::outpostAndScore);
        autoChooser.addRoutine("Left", this::left);
        autoChooser.addRoutine("Left2", this::left2);
        autoChooser.addRoutine("Outpost Only", this::outpostOnly);

        SmartDashboard.putData("Auton Selector", autoChooser);
        SmartDashboard.putBoolean("astop", false);
        RobotModeTriggers.autonomous()
            .whileTrue(autoChooser.selectedCommandScheduler()
                .unless(astop)  
        );

        RobotModeTriggers.teleop().or(astop).onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
    }

    public AutoRoutine left() {
        var path = AutoConsts.LEFT.poses;

        var sequence = Commands.sequence(
            Commands.runOnce(() -> drivetrain.resetPose(AutoConsts.colorPose(AutoConsts.LEFT.INIT_POSE))),
            drivetrain.goToPoseCommand(() -> AutoConsts.colorPose(path[0]))
                .until(() -> drivetrain.isAtPoseSetpoint(false))
                .andThen(drivetrain.goToPoseCommand().withTimeout(1)),
            rcontainer.shoot()
        );

        var routine = autoFactory.newRoutine("left");
        routine.active().onTrue(sequence);
        return routine;
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

        // rightToOutpost.atTime("collect").onTrue();

        routine.active().onTrue(
            Commands.parallel(
                // Commands.runOnce(() -> intake.deployed = true),
                // Commands.runOnce(() -> intake.deployed = false),
                Commands.runOnce(() -> drivetrain.speedMultiplier = 0.2),

                Commands.sequence(
                    Commands.runOnce(() -> intake.deployed = true),
                    Commands.runOnce(() -> SmartDashboard.putString("astage", "zeroth")),
                    autoFactory.resetOdometry("rightToOutpost_part1"),
                    // rightToOutpost.cmd(),
                    Commands.runOnce(() -> SmartDashboard.putString("astage", "first")),
                    drivetrain.goToPoseCommand(() -> rightToOutpost.getFinalPose().get())
                        .until(() -> drivetrain.isAtPoseSetpoint(false)),
                    // Commands.runOnce(() -> drivetrain.setControl(new SwerveRequest.Idle())),
                    Commands.runOnce(() -> SmartDashboard.putString("astage", "second")),
                    Commands.waitSeconds(3),
                    Commands.runOnce(() -> SmartDashboard.putString("astage", "third")),

                    Commands.runOnce(() -> intake.deployed = false),

                    Commands.race(
                        drivetrain.goToPoseCommand(() -> Locator.getInstance().extentionPose),
                        Commands.waitSeconds(2.5)
                    ),
                    Commands.parallel(
                        intake.runAtSpeed(RPM.of(3000)),
                        Commands.race(
                            rcontainer.shoot(),
                            Commands.waitSeconds(10)
                        ).andThen(rcontainer.idleAll())
                    ),
                    Commands.parallel(indexer.idle(),
                        shooter.idle()),
                    Commands.runOnce(() -> intake.deployed = true)
                )

            )
        );
        return routine;
    }

    public AutoRoutine outpostAndScore() {
        var path = AutoConsts.OPAS.poses;

        var sequence = Commands.sequence(
            // Commands.runOnce(() -> drivetrain.resetPose(AutoConsts.OPAS.INIT_POSE)),
            // drivetrain.goToPoseCommand(() -> AutoConsts.OPAS.path[0])
            //     .until(() -> drivetrain.isAtPoseSetpoint(false))
            //     .andThen(drivetrain.goToPoseCommand().withTimeout(5)),

            // drivetrain.goToPoseCommand(() -> path[1])
            //     .until(() -> drivetrain.isAtPoseSetpoint(false))
        );

        var routine = autoFactory.newRoutine("Outpost and Score");
        routine.active().onTrue(sequence);
        return routine;
    }

    public AutoRoutine outpostAndScore1() {
        Pose2d INIT_POSE = new Pose2d(
                            3.709699869155884,
                            1.9885972738265991, Rotation2d.fromDegrees(90));

        var sequence = Commands.sequence();

        var routine = autoFactory.newRoutine("Bump");
        routine.active()
            .onTrue(
                Commands.sequence(
                    Commands.runOnce(() -> drivetrain.resetPose(INIT_POSE)),
                    // drivetrain.goToPoseCommand(() -> INIT_POSE),
                    Commands.runOnce(() -> SmartDashboard.putString("autostage", "stage 0")),
                    // Commands.waitSeconds(2),
                    Commands.runOnce(() -> SmartDashboard.putString("autostage", "done waitng")),
                    drivetrain.goToPoseCommand(() -> new Pose2d(
                        2.059431552886963,
                        2.451367139816284, Rotation2d.fromDegrees(90))
                    ),
                    drivetrain.pointAtPose(() -> Locator.getInstance().hubPose),
                    drivetrain.goToPoseCommand(() -> new Pose2d(
                        8.683216094970703,
                        2.8809804916381836,
                        Rotation2d.fromDegrees(90)
                    )),
                    Commands.runOnce(() -> SmartDashboard.putString("autostage", "across field")),
                    // drivetrain.pointAtPose(Constants.hubPose),
                    drivetrain.goToPoseCommand(() -> new Pose2d(
                        8.4885172843933105,
                        5.456998348236084,
                        Rotation2d.fromDegrees(90)
                    )),
                    // drivetrain.pointAtPose(Constants.hubPose),
                    drivetrain.goToPoseCommand(() -> new Pose2d(
                        1.6959257125854492,
                        5.471975326538086,
                        new Rotation2d()
                    )),
                    drivetrain.pointAtPose(() -> Locator.getInstance().hubPose),
                    drivetrain.pointAtPose(() -> Locator.getInstance().hubPose)
                )
            );
        return routine;
    }
}