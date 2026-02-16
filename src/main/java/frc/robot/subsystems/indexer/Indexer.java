package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Indexer extends SubsystemBase {
    public SmartMotorControllerConfig baseMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  // Feedforward Constants
  .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));
    ;

    public FlyWheelConfig baseRollerConfig = new FlyWheelConfig()
        // .withMOI(KilogramSquareMeters.of(0.0006203525)) // TODO: this MOI is bullcrap
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(-6000))
    ;
        // .withClosedLoopController(0, 0, 0)
        // .withSimClosedLoopController(0, 0, 0)

        // .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))

        // .withGearing(new MechanismGearing(GearBox.fromStages("20:24")))
        

    // private SmartMotorControllerConfig ceilingMotorConfig = baseMotorConfig.clone()
    //     .withClosedLoopController(0, 0, 0)
    //     .withSimClosedLoopController(0, 0, 0)

    //     .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    //     .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))

    //     .withGearing(new MechanismGearing(GearBox.fromStages("20:24")))
    //     ;

    // private SmartMotorControllerConfig kickerMotorConfig = baseMotorConfig.clone()
    //     .withClosedLoopController(0, 0, 0)
    //     .withSimClosedLoopController(0, 0, 0)

    //     .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
    //     .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))

    //     .withGearing(new MechanismGearing(GearBox.fromStages("36:24")))
    //     ;

    // private Roller floor = new Roller("floor", Constants.Indexer.floorMotorID, floorMotorConfig, baseRollerConfig);

    // private Roller kicker = new Roller("kicker", Constants.Indexer.kickerMotorID, kickerMotorConfig, baseRollerConfig);

    public Indexer() {

    }
}
