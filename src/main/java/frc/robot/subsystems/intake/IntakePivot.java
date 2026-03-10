package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakePivot extends SubsystemBase {
    private TalonFX pivotMotor = new TalonFX(30);
    private CANcoder pivotEncoder = new CANcoder(32);

    private final SmartMotorControllerConfig pivotMotorConfig =
            new SmartMotorControllerConfig(this)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withGearing(
                            new MechanismGearing(
                                    GearBox.fromStages(
                                            "4:1", "4:1",
                                            "12:48"))) // , Sprocket.fromStages("4:1")))
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    // .withSimClosedLoopController(5.5, 0, 1)
                    // .withFeedforward(new ArmFeedforward(0, 1.013332000, 1))
                    .withClosedLoopController(20, 0, 0)
                    .withFeedforward(new ArmFeedforward(0, 0, 0))
                    .withEncoderInverted(false)
                    .withExternalEncoder(pivotEncoder)
                    .withExternalEncoderGearing(1)
                    // .withExternalEncoderZeroOffset(Degrees.of(0))
                    .withUseExternalFeedbackEncoder(true)
                    .withMotorInverted(true)
                    .withTelemetry(
                            "YAMSIntakePivotMotor", Constants.getAppropriateTelemetryLevel());

    private final SmartMotorController pivotMotorController =
            new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX60(1), pivotMotorConfig);

    private ArmConfig pivotConfig =
            new ArmConfig(pivotMotorController)
                    .withHardLimit(Degrees.of(-10.586006), Degrees.of(90 + 32.086608))
                    .withStartingPosition(Degrees.of(0))
                    .withLength(Inches.of(13.060457))
                    .withMOI(KilogramSquareMeters.of(0.0535991403))
                    .withTelemetry("YAMSIntakePivotMech", Constants.getAppropriateTelemetryLevel());

    public Constants.Intake.PivotState state = Constants.Intake.PivotState.HalfDeploy;

    public Arm pivot = new Arm(pivotConfig);

    public IntakePivot() {
        setName("intakePivot");
        setDefaultCommand(defaultcmd());
    }

    private Command defaultcmd() {
        return pivot.setAngle(() -> state.angle);
    }

    public Command setState(Constants.Intake.PivotState state) {
        return Commands.runOnce(() -> this.state = state);
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
