package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveRobotOriented extends Command {
    private Drivetrain drivetrain;
    private LinearVelocity vx;
    private LinearVelocity vy;
    private AngularVelocity vtheta;
    private boolean killOnEnd;

    public DriveRobotOriented(Drivetrain drivetrain,
        boolean killOnEnd,
        LinearVelocity vx,
        LinearVelocity vy,
        AngularVelocity vtheta
    ) {
        this.drivetrain = drivetrain;
        this.killOnEnd = killOnEnd;
        this.vx = vx;
        this.vy = vy;
        this.vtheta = vtheta;
        addRequirements(drivetrain);
    }

    public DriveRobotOriented(Drivetrain drivetrain,
        LinearVelocity vx,
        LinearVelocity vy,
        AngularVelocity vtheta
    ) {
        this.drivetrain = drivetrain;
        this.killOnEnd = true;
        this.vx = vx;
        this.vy = vy;
        this.vtheta = vtheta;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        drivetrain.setControl(
            new SwerveRequest.RobotCentric()
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(vtheta)
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (killOnEnd) {
            drivetrain.setControl(new SwerveRequest.Idle());
        }
    }
}