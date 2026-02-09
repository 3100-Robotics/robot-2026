package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Indexer extends SubsystemBase {
    // floorMotor is the main motor that the others follow
    private SparkMax rawFloorMotor = new SparkMax(40, MotorType.kBrushless);
    private SparkMax rawCeilingMotor  = new SparkMax(41, MotorType.kBrushless);
    private SparkMax rawKickerMotor = new SparkMax(42, MotorType.kBrushless);

    private SmartMotorControllerConfig rootConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withStatorCurrentLimit(Amps.of(40))

        // Feedback Constants (PID Constants)
        .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // Telemetry name and verbosity level
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        // Motor properties to prevent over currenting.
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
    ;

    private SmartMotorControllerConfig floorMotorConfig = new SmartMotorControllerConfig();


    public Indexer() {

    }
}
