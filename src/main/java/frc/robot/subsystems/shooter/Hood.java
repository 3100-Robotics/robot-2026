package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import java.time.Period;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

        .withGearing(new MechanismGearing(GearBox.fromStages("4:1", "182:10")))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withSimClosedLoopController(50, 0, 0)
        .withFeedforward(new ArmFeedforward(0, 0, 0))

        .withMotorInverted(true)

        .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
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

    private Command setAngle = hood.setAngle(Degrees.of(20)).withName("angle 1");
    private Command setAngle2 = hood.setAngle(Degrees.of(65)).withName("angle 2");

    public Hood() {

    }

    @Override
    public void periodic() {
        hood.getMechanismSetpoint().ifPresent(setpoint -> SmartDashboard.putNumber("hoodSetpoint", setpoint.in(Degrees)));
        // SmartDashboard.putNumber("wrealAngle", hood.getAngle().in(Degrees));

        // SmartDashboard.putData(setAngle);
        // SmartDashboard.putData(setAngle2);

        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }

    // SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    //     .withClosedLoopController(10, 0, 0)//, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
    //     .withFeedforward(new ArmFeedforward(0, 0.865000, 50, 0))
    //     .withSoftLimit(Degrees.of(-30), Degrees.of(100))
    //     .withGearing(new MechanismGearing(0.01373626373))
    //     .withIdleMode(MotorMode.COAST)
    //     .withTelemetry("hoodMOTOR", TelemetryVerbosity.HIGH)
    //     .withStatorCurrentLimit(Amps.of(40))
    //     .withMotorInverted(false)
    //     // .withClosedLoopRampRate(Seconds.of(0.25))
    //     // .withOpenLoopRampRate(Seconds.of(0.25))
    //     .withControlMode(ControlMode.CLOSED_LOOP);
    // SmartMotorController smartMotorController = new TalonFXWrapper(armMotor,
    //                                                                 DCMotor.getKrakenX60(1),
    //                                                                 motorConfig);
    // ArmConfig armCfg = new ArmConfig(smartMotorController)
    //     .withLength(Inches.of(8.900512))
    //     .withMOI(KilogramSquareMeters.of(0.0190245794))
    //     .withHardLimit(Degrees.of(-100), Degrees.of(200))
    //     .withStartingPosition(Degrees.of(0)); // Parallel to the ground
    //     ;

    // private Arm arm = new Arm(armCfg);

    // private Command setAngle2 = setAngle(Degrees.of(90)).withName("Arm Angle 2");
    // private Command setAngle1 = setAngle(Degrees.of(0)).withName("Arm Angle 1");

    // public Hood() {
    //     // setDefaultCommand(setAngle1);
    // }

    // public Command setAngle(Angle angle) { return arm.setAngle(angle);}
    // public Command set(double dutycycle) { return arm.set(dutycycle);}

    // @Override
    // public void periodic() {
    //     SmartDashboard.putData(setAngle2);
    //     SmartDashboard.putData(setAngle1);
    //     arm.updateTelemetry();
    // }

    // @Override
    // public void simulationPeriodic() {
    //     // This method will be called once per scheduler run during simulation
    //     arm.simIterate();
    // }
}
