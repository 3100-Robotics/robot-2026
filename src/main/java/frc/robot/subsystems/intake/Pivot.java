package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
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
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Pivot extends SubsystemBase {
    private TalonFX pivotMotor = new TalonFX(30);

    private final SmartMotorControllerConfig pivotMotorConfig = new SmartMotorControllerConfig(this)
        .withStatorCurrentLimit(Amps.of(60))
        .withSupplyCurrentLimit(Amps.of(60))

        .withGearing(new MechanismGearing(GearBox.fromStages("4:1")))//, Sprocket.fromStages("4:1")))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withSimClosedLoopController(5.5, 0, 1)
        .withFeedforward(new ArmFeedforward(0, 1.012000, 1))

        .withMotorInverted(false)

        .withTelemetry(Constants.Intake.telemetryYAMSPivot+"Motor", Constants.getAppropriateTelemetryLevel())
        ;
    
    private final SmartMotorController pivotMotorController = new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX60(1), pivotMotorConfig);

    private ArmConfig pivotConfig = new ArmConfig(pivotMotorController)
        .withHardLimit(
            Degrees.of(-10.586006),
            Degrees.of(90+32.086608))
        .withStartingPosition(Degrees.of(0))

        .withLength(Inches.of(13.060457))
        .withMOI(KilogramSquareMeters.of(0.0535991403))

        .withTelemetry(Constants.Intake.telemetryYAMSPivot+"Mech", Constants.getAppropriateTelemetryLevel());

    // Arm Mechanism
    private Arm pivot = new Arm(pivotConfig);

    private Command setAngle = pivot.setAngle(Degrees.of(20)).withName("angle 1");
    private Command setAngle2 = pivot.setAngle(Degrees.of(65)).withName("angle 2");

    public Pivot() {
        Logging.registerDebugCommand(Constants.Intake.telemetryNamePivot+"dbgAngle0", setAngle);
        Logging.registerDebugCommand(Constants.Intake.telemetryNamePivot+"dbgAngle1", setAngle2);
    }

    @Override
    public void periodic() {
        pivot.getMechanismSetpoint().ifPresent(setpoint -> SmartDashboard.putNumber(Constants.Intake.telemetryNamePivot+"setpoint", setpoint.in(Degrees)));
        pivot.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }
}
