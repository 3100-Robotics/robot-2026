package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterHood extends SubsystemBase {
    private TalonFX armMotor = new TalonFX(Constants.Shooter.hoodMotorID);

    private final SmartMotorControllerConfig armMotorConfig = new SmartMotorControllerConfig(this)
        .withStatorCurrentLimit(Amps.of(50))
        .withSupplyCurrentLimit(Amps.of(40))

        .withIdleMode(MotorMode.COAST)

        .withGearing(new MechanismGearing(GearBox.fromStages("48:12", "182:10")))
        .withControlMode(ControlMode.CLOSED_LOOP)

        .withClosedLoopController(530, 0, 0)
        .withSimClosedLoopController(5, 0, 0)
        .withFeedforward(new ArmFeedforward(140, 0, 0))
        .withSimFeedforward(new ArmFeedforward(0, 0, 0))

        .withMotorInverted(true)

        .withTelemetry(Constants.Shooter.YAMS.nameHood, Constants.getAppropriateTelemetryLevel())
        ;

    private final SmartMotorController armMotorController = new TalonFXWrapper(armMotor, DCMotor.getKrakenX60(1), armMotorConfig);

    private ArmConfig hoodConfig = new ArmConfig(armMotorController)
        .withHardLimit(Degrees.of(12.667292), Degrees.of(40))
        // Starting position is where your arm starts
        .withStartingPosition(Degrees.of(12.667292))
        // Length and mass of your arm for sim.
        .withLength(Inches.of(8.900512))
        .withMOI(KilogramSquareMeters.of(0.0190245794))
        // Telemetry name and verbosity for the arm.
        .withTelemetry("HoodMech", Constants.getAppropriateTelemetryLevel());

    // Arm Mechanism
    public Arm hood = new Arm(hoodConfig);

    public ShooterHood() {
        setName("shooterHood");
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
