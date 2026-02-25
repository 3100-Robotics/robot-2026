package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
import frc.robot.Locator;
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

    public Auton(
        Drivetrain drivetrain,
        Shooter shooter,
        Indexer indexer,
        Intake intake
    ) {
        // autoChooser.addRoutine();
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

        autoChooser.addRoutine("Outpose and Score", this::outpostAndScore);

        SmartDashboard.putData("Auton Selector", autoChooser);

        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    public AutoRoutine outpostAndScore() {
        var path = AutoConsts.OPAS.poses;

        var sequence = Commands.sequence(
            Commands.runOnce(() -> drivetrain.resetPose(AutoConsts.OPAS.INIT_POSE)),
            drivetrain.goToPoseCommand(() -> path[0])
                .until(() -> drivetrain.isAtPoseSetpoint(false))
                .andThen(drivetrain.goToPoseCommand().withTimeout(5)),

            drivetrain.goToPoseCommand(() -> path[1])
                .until(() -> drivetrain.isAtPoseSetpoint(false))
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