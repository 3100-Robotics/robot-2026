package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.math.SpeedSet;

public class Indexer extends SubsystemBase {
    public AngularVelocity kickerSpeed = RPM.of(0);
    public AngularVelocity ceilingSpeed = RPM.of(0);
    public AngularVelocity floorSpeed = RPM.of(0);

    private Supplier<AngularVelocity> kickerSpeedProvider = () -> kickerSpeed;
    private Supplier<AngularVelocity> ceilingSpeedProvider = () -> ceilingSpeed;
    private Supplier<AngularVelocity> floorSpeedProvider = () -> floorSpeed;

    private SparkMax kickerMotor = new SparkMax(Constants.Indexer.kickerMotorID, MotorType.kBrushless);
    private SparkMax ceilingMotor = new SparkMax(Constants.Indexer.ceilingMotorID, MotorType.kBrushless);
    private TalonFX floorMotor = new TalonFX(Constants.Indexer.floorMotorID);

    final VelocityVoltage floorRequest = new VelocityVoltage(0).withSlot(0);

    private SparkBaseConfig sparkTypeBaseConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
    ;

    private SparkBaseConfig kickerConfig = new SparkMaxConfig().apply(sparkTypeBaseConfig)
        .inverted(false)
        .apply(
            new ClosedLoopConfig()
            .p(0.01)
            .apply(new FeedForwardConfig()
                .kV(0.2)
            )
        );

    private SparkBaseConfig ceilingConfig = new SparkMaxConfig().apply(sparkTypeBaseConfig)
        .inverted(false)
        .apply(
            new ClosedLoopConfig()
            .p(0.1)
            .apply(new FeedForwardConfig()
                .kV(0.12)
            )
        );

    private TalonFXConfiguration floorConfig = new TalonFXConfiguration()
        .withAudio(
            new AudioConfigs()
            .withBeepOnBoot(true)
            .withBeepOnConfig(true)
        )
        .withMotorOutput(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        )
        .withSlot0(
            new Slot0Configs()
            .withKP(0.01)
            .withKV(0.15)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(40))
            .withSupplyCurrentLimit(Amps.of(40))
        )
    ;

    public Indexer() {
        kickerMotor.configure(kickerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        ceilingMotor.configure(ceilingConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        floorMotor.getConfigurator().apply(floorConfig);
        setDefaultCommand(runToSetpoints());
    }

    public Command setSetpointsOfRollers(SpeedSet velocities) {
        return Commands.runOnce(() -> {
            kickerSpeed = velocities.kickerSpeed
                .times(Constants.Indexer.kickerRatioRecip);
            ceilingSpeed = velocities.ceilingSpeed
                .times(Constants.Indexer.ceilingRatioRecip);
            floorSpeed = velocities.floorSpeed
                .times(Constants.Indexer.floorRatioRecip);
        });
    }

    private Command runToSetpoints() {
        return run(() -> {
            kickerMotor.getClosedLoopController().setSetpoint(kickerSpeedProvider.get().in(RPM), ControlType.kVelocity);
            ceilingMotor.getClosedLoopController().setSetpoint(ceilingSpeedProvider.get().in(RPM), ControlType.kVelocity);
            floorMotor.setControl(floorRequest.withVelocity(floorSpeedProvider.get().in(RotationsPerSecond)));
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("2kickerRPM", 
            kickerMotor.getEncoder().getVelocity()
            * Constants.Indexer.kickerRatio
        );
        SmartDashboard.putNumber("2ceilingRPM", 
            ceilingMotor.getEncoder().getVelocity()
            * Constants.Indexer.ceilingRatio
        );
        SmartDashboard.putNumber("2floorRPM", 
            floorMotor.getVelocity().getValue().in(RPM)
            * Constants.Indexer.floorRatio
        );

        SmartDashboard.putNumber("2kickerCurrent", kickerMotor.getOutputCurrent());
        SmartDashboard.putNumber("2ceilingCurrent", ceilingMotor.getOutputCurrent());
        SmartDashboard.putNumber("2floorCurrent", floorMotor.getStatorCurrent().getValue().in(Amps));
    }
}
