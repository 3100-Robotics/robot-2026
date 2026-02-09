package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logging;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

/**
* Class for the rollers on the intake
*/
public class Roller extends SubsystemBase {
    private SparkMax rawRollerMotor = new SparkMax(Constants.Intake.rollerMotorID, MotorType.kBrushless);

    private SmartMotorControllerConfig rollerMotorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        // .withClosedLoopController(0, 0, 0, RPM.of(6000), DegreesPerSecondPerSecond.of(90))
        .withSimClosedLoopController(0.0001, 0, 0)//, RPM.of(6000), DegreesPerSecondPerSecond.of(90))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0.19, 0))
        // Telemetry name and verbosity level
        .withTelemetry(Constants.Intake.telemetryYAMSRoller+"Motor", Constants.getAppropriateTelemetryLevel())
        .withGearing(new MechanismGearing(GearBox.fromStages("3:2")))
        // Motor properties to prevent over currenting.
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));
        
    private SmartMotorController rollerMotorController = new SparkWrapper(rawRollerMotor, DCMotor.getNEO(1), rollerMotorConfig);

    private final FlyWheelConfig rollerConfig = new FlyWheelConfig(rollerMotorController)
        .withMOI(KilogramSquareMeters.of(0.0006203525))
        .withUpperSoftLimit(RPM.of(6000))
        .withTelemetry(Constants.Intake.telemetryYAMSRoller+"Mech", Constants.getAppropriateTelemetryLevel());

    private FlyWheel roller = new FlyWheel(rollerConfig);

    private Command dbgUp = roller.setSpeed(RPM.of(3000)).withName("dbgUp");
    private Command dbgDown = roller.set(0).andThen(Commands.waitSeconds(0.01)).andThen(roller.setSpeed(RPM.of(0))).withName("dbgDown");


    public Roller() {
        Logging.registerDebugCommand(Constants.Intake.telemetryNameRoller+"dbgUp", dbgUp);
        Logging.registerDebugCommand(Constants.Intake.telemetryNameRoller+"dbgDown", dbgDown);
    }

    @Override
    public void periodic() {
        roller.getMotorController().getMechanismSetpointVelocity()
            .ifPresent(
                setpoint -> SmartDashboard.putNumber(Constants.Intake.telemetryNameRoller+"RPM_Setpoint", setpoint.in(RPM))
            );
        SmartDashboard.putNumber(Constants.Intake.telemetryNameRoller+"RPM_Speed", roller.getSpeed().in(RPM));
        roller.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        roller.simIterate();
    }
}
