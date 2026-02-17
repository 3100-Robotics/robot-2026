package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logging;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Hood extends SubsystemBase {
    private TalonFX armMotor = new TalonFX(50);

    private final SmartMotorControllerConfig armMotorConfig = new SmartMotorControllerConfig(this)
        .withStatorCurrentLimit(Amps.of(40))
        .withSupplyCurrentLimit(Amps.of(40))

        .withGearing(new MechanismGearing(GearBox.fromStages("48:12", "182:10")))
        .withControlMode(ControlMode.CLOSED_LOOP)

        .withClosedLoopController(50, 0, 0)
        .withSimClosedLoopController(50, 0, 0)
        .withFeedforward(new ArmFeedforward(0, 0, 0))
        .withSimFeedforward(new ArmFeedforward(0, 0, 0))

        .withMotorInverted(true)

        .withTelemetry(Constants.Shooter.YAMS.nameHood, TelemetryVerbosity.HIGH)
        ;
    
    private final SmartMotorController armMotorController = new TalonFXWrapper(armMotor, DCMotor.getKrakenX60(1), armMotorConfig);

    private ArmConfig hoodConfig = new ArmConfig(armMotorController)
        .withHardLimit(Degrees.of(12.667292), Degrees.of(12.667292+66.617755))
        // Starting position is where your arm starts
        .withStartingPosition(Degrees.of(13))
        // Length and mass of your arm for sim.
        .withLength(Inches.of(8.900512))
        .withMOI(KilogramSquareMeters.of(0.0190245794))
        // Telemetry name and verbosity for the arm.
        .withTelemetry("HoodMech", TelemetryVerbosity.HIGH);

    // Arm Mechanism
    private Arm hood = new Arm(hoodConfig);

    private Command dbg_angle_0 = hood.setAngle(Degrees.of(20)).withName("angle 1");
    private Command dbg_angle_1 = hood.setAngle(Degrees.of(50)).withName("angle 2");

    public Supplier<Angle> angleTargetProvider = () -> Degrees.of(0);
    public Command continuousAngle = hood.setAngle(angleTargetProvider);
    public Command stop = hood.runTo(hood.getAngle(), Degrees.of(90)).andThen(hood.set(0));

    public Hood(Supplier<Angle> angle) {
        setName("shooterHood");
        angleTargetProvider = angle;
        Logging.registerDebugCommand(
            Constants.join('/', Constants.Shooter.Main.nameHood, dbg_angle_0.getName()), dbg_angle_0);
        Logging.registerDebugCommand(
            Constants.join('/', Constants.Shooter.Main.nameHood, dbg_angle_1.getName()), dbg_angle_1);
    }

    @Override
    public void periodic() {
        hood.getMechanismSetpoint().ifPresent(setpoint -> SmartDashboard.putNumber("hoodSetpoint", setpoint.in(Degrees)));

        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
