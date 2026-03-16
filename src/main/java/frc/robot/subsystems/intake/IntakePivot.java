package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import java.util.Optional;

import com.ctre.phoenix6.hardware.CANcoder;
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

@SuppressWarnings("unused")
public class IntakePivot extends SubsystemBase {
    private TalonFX pivotMotor = new TalonFX(30);
    private CANcoder pivotEncoder = new CANcoder(32);

    private final SmartMotorControllerConfig pivotMotorConfig = new SmartMotorControllerConfig(this)
        .withStatorCurrentLimit(Amps.of(40))
        .withSupplyCurrentLimit(Amps.of(40))

        .withGearing(new MechanismGearing(GearBox.fromStages("4:1", "4:1", "12:48")))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(40,0,0)
        .withFeedforward(new ArmFeedforward(0, 0, 0))

        .withEncoderInverted(false)
        .withExternalEncoder(pivotEncoder)
        .withExternalEncoderGearing(1)
        .withUseExternalFeedbackEncoder(true)

        .withMotorInverted(false)

        .withTelemetry(Constants.Intake.telemetryYAMSPivot+"Motor", Constants.getAppropriateTelemetryLevel())
    ;
    
    private final SmartMotorController pivotMotorController = new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX60(1), pivotMotorConfig);

    private ArmConfig pivotConfig = new ArmConfig(pivotMotorController)
        .withHardLimit(Degrees.of(-10.586006), Degrees.of(90+32.086608))
        .withStartingPosition(Degrees.of(0))

        .withLength(Inches.of(13.060457))
        .withMOI(KilogramSquareMeters.of(0.0535991403))

        .withTelemetry(Constants.Intake.telemetryYAMSPivot+"Mech", Constants.getAppropriateTelemetryLevel())
    ;

    public Arm pivot = new Arm(pivotConfig);

    private Command dbg_angle_0 = pivot.setAngle(Constants.Intake.pivotDeployAngle).withName("dbg_angle_0");
    private Command dbg_angle_1 = pivot.setAngle(Constants.Intake.pivotStowAngle).withName("dbg_angle_1");

    public IntakePivot() {
        setName("intakePivot");
        Logging.registerDebugCommand(Constants.Intake.telemetryNamePivot+dbg_angle_0.getName(), dbg_angle_0);
        Logging.registerDebugCommand(Constants.Intake.telemetryNamePivot+dbg_angle_1.getName(), dbg_angle_1);
    }

    @Override
    public void periodic() {
        pivot.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }
}
