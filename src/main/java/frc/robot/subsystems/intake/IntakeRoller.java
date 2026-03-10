package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Supplier;

public class IntakeRoller extends SubsystemBase {
    private SparkBaseConfig rollerConfig =
            new SparkMaxConfig()
                    .idleMode(IdleMode.kCoast)
                    .smartCurrentLimit(40)
                    .inverted(false)
                    .apply(new ClosedLoopConfig().p(0.03).apply(new FeedForwardConfig().kV(0.3)));

    private SparkMax rollerMotor =
            new SparkMax(Constants.Intake.rollerMotorID, MotorType.kBrushless);

    public Constants.Intake.RollerState state = Constants.Intake.RollerState.Off;

    private AngularVelocity rollerSpeed = state.speed;
    private Supplier<AngularVelocity> rollerSpeedProvider = () -> rollerSpeed;

    public IntakeRoller() {
        rollerMotor.configure(
                rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        setDefaultCommand(runToSetpoint());
    }

    public Command setState(Constants.Intake.RollerState state) {
        return Commands.runOnce(
                () -> {
                    this.state = state;
                    switch (state) {
                        case Off:
                            this.rollerSpeed = RPM.of(0);
                            setDefaultCommand(stop());
                            break;
                        case Normal:
                            this.rollerSpeed = rollerSpeed.times(Constants.Intake.rollerRatioRecip);
                            setDefaultCommand(runToSetpoint());
                            break;
                    }
                });
    }

    private Command runToSetpoint() {
        return run(
                () -> {
                    rollerMotor
                            .getClosedLoopController()
                            .setSetpoint(rollerSpeedProvider.get().in(RPM), ControlType.kVelocity);
                });
    }

    private Command stop() {
        return run(
                () -> {
                    rollerMotor.set(0);
                });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
                "2rollerRPM",
                rollerMotor.getEncoder().getVelocity() * Constants.Intake.rollerRatio);

        SmartDashboard.putNumber("2rollerCurrent", rollerMotor.getOutputCurrent());
    }
}
