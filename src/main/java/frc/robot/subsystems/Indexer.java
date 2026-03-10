package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.math.SpeedSet;
import java.util.function.Supplier;

public class Indexer extends SubsystemBase {
    public AngularVelocity kickerSpeed = RPM.of(0);
    public AngularVelocity ceilingSpeed = RPM.of(0);
    public AngularVelocity floorSpeed = RPM.of(0);

    private Supplier<AngularVelocity> kickerSpeedProvider = () -> kickerSpeed;
    private Supplier<AngularVelocity> ceilingSpeedProvider = () -> ceilingSpeed;
    private Supplier<AngularVelocity> floorSpeedProvider = () -> floorSpeed;

    private SparkMax kickerMotor =
            new SparkMax(Constants.Indexer.kickerMotorID, MotorType.kBrushless);
    private SparkMax ceilingMotor =
            new SparkMax(Constants.Indexer.ceilingMotorID, MotorType.kBrushless);
    private TalonFX floorMotor = new TalonFX(Constants.Indexer.floorMotorID);

    final VelocityVoltage floorRequest = new VelocityVoltage(0).withSlot(0);

    private SparkBaseConfig sparkTypeBaseConfig =
            new SparkMaxConfig().idleMode(IdleMode.kCoast).smartCurrentLimit(40);

    private SparkBaseConfig kickerConfig =
            new SparkMaxConfig()
                    .apply(sparkTypeBaseConfig)
                    .inverted(false)
                    .apply(new ClosedLoopConfig().p(0.00001).apply(new FeedForwardConfig().kV(0.0013845)));

    private SparkBaseConfig ceilingConfig =
            new SparkMaxConfig()
                    .apply(sparkTypeBaseConfig)
                    .inverted(false)
                    .apply(new ClosedLoopConfig().p(0.1).apply(new FeedForwardConfig().kV(0.12)));

    private TalonFXConfiguration floorConfig =
            new TalonFXConfiguration()
                    .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true))
                    .withMotorOutput(
                            new MotorOutputConfigs()
                                    .withInverted(InvertedValue.Clockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Coast))
                    .withSlot0(new Slot0Configs().withKP(0.01).withKV(0.15))
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(Amps.of(40))
                                    .withSupplyCurrentLimit(Amps.of(40)));

    private final DCMotor kickerGearbox = DCMotor.getNEO(1);
    private final SparkClosedLoopController kickerController = kickerMotor.getClosedLoopController();
    private final RelativeEncoder kickerEncoder = kickerMotor.getEncoder();
    private final SparkMaxSim kickerMotorSim = new SparkMaxSim(kickerMotor, kickerGearbox);
    private final FlywheelSim kickerSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(kickerGearbox, 0.0001, Constants.Indexer.kickerRatio),
        kickerGearbox
    );

    private final DCMotor ceilingGearbox = DCMotor.getNEO(1);
    private final SparkClosedLoopController ceilingController = ceilingMotor.getClosedLoopController();
    private final RelativeEncoder ceilingEncoder = ceilingMotor.getEncoder();
    private final SparkMaxSim ceilingMotorSim = new SparkMaxSim(ceilingMotor, ceilingGearbox);
    private final FlywheelSim ceilingSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(ceilingGearbox, 0.0001, Constants.Indexer.ceilingRatio),
        ceilingGearbox
    );

    private final DCMotor floorGearbox = DCMotor.getKrakenX60(1);

    // private final SparkClosedLoopController floorController = floorMotor.getClosedLoopController();
    // private final RelativeEncoder floorEncoder = floorMotor.getEncoder();

    private final TalonFXSimState floorMotorSim = floorMotor.getSimState();
    private final FlywheelSim floorSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(kickerGearbox, 0.0001, Constants.Indexer.kickerRatio),
        kickerGearbox
    );

    public Indexer() {
        kickerMotor.configure(
                kickerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        ceilingMotor.configure(
                ceilingConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        floorMotor.getConfigurator().apply(floorConfig);
        setDefaultCommand(runToSetpoints());
    }

    @Override
    public void simulationPeriodic() {
        double timestep = 20e-3;
        var inputVoltage = RobotController.getInputVoltage();

        kickerSim.setInputVoltage(kickerMotor.getAppliedOutput() * inputVoltage);
        kickerSim.update(timestep);
        kickerMotorSim.iterate(kickerSim.getAngularVelocityRPM(), inputVoltage, timestep);

        ceilingSim.setInputVoltage(ceilingMotor.getAppliedOutput() * inputVoltage);
        ceilingSim.update(timestep);
        ceilingMotorSim.iterate(ceilingSim.getAngularVelocityRPM(), inputVoltage, timestep);

        floorSim.setInputVoltage(floorMotor.getDutyCycle().getValueAsDouble() * inputVoltage);
        floorSim.update(timestep);
        floorMotorSim.setRawRotorPosition(floorMotorSim.getAngularPosition().times(kGearRatio));
        floorMotorSim.setRotorVelocity(floorMotorSim.().times(kGearRatio));
        // floorMotorSim.
        // floorMotorSim.iterate(floorSim.getAngularVelocityRPM(), inputVoltage, timestep);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(
                kickerMotor.getOutputCurrent(),
                ceilingMotor.getOutputCurrent(),
                floorMotor.getStatorCurrent().getValueAsDouble()
            )
        );

    }

    public Command setSetpointsOfRollers(SpeedSet velocities) {
        return Commands.runOnce(
                () -> {
                    kickerSpeed = velocities.kickerSpeed.times(Constants.Indexer.kickerRatio);
                    ceilingSpeed = velocities.ceilingSpeed.times(Constants.Indexer.ceilingRatio);
                    floorSpeed = velocities.floorSpeed.times(Constants.Indexer.floorRatio);
                });
    }

    private Command runToSetpoints() {
        return run(
                () -> {
                    if (kickerSpeedProvider.get().in(RPM) == 0) {
                        kickerMotor.set(0);
                    } else {
                        kickerMotor
                            .getClosedLoopController()
                            .setSetpoint(kickerSpeedProvider.get().in(RPM), ControlType.kVelocity);
                    }

                    if (ceilingSpeedProvider.get().in(RPM) == 0) {
                        ceilingMotor.set(0);
                    } else {
                        ceilingMotor
                            .getClosedLoopController()
                            .setSetpoint(ceilingSpeedProvider.get().in(RPM), ControlType.kVelocity);
                    }

                    floorMotor.setControl(
                        floorRequest.withVelocity(
                            floorSpeedProvider.get().in(RotationsPerSecond)
                        )
                    );
                });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
                "2kickerRPM",
                kickerMotor.getEncoder().getVelocity() * Constants.Indexer.kickerRatioRecip);
        SmartDashboard.putNumber(
                "2kickerMotorRPM",
                kickerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber(
                "2ceilingRPM",
                ceilingMotor.getEncoder().getVelocity() * Constants.Indexer.kickerRatioRecip);
        SmartDashboard.putNumber(
                "2floorRPM",
                floorMotor.getVelocity().getValue().in(RPM) * Constants.Indexer.kickerRatioRecip);

        SmartDashboard.putNumber("2kickerCurrent", kickerMotor.getOutputCurrent());
        SmartDashboard.putNumber("2ceilingCurrent", ceilingMotor.getOutputCurrent());
        SmartDashboard.putNumber(
                "2floorCurrent", floorMotor.getStatorCurrent().getValue().in(Amps));
    }
}
