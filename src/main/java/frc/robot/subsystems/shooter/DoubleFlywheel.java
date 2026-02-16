package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class DoubleFlywheel extends SubsystemBase {
    private final DCMotor GEARBOX = DCMotor.getNEO(2);

    private final int flywheelIndex;

    private SparkMax vendorLead;
    private SparkMax vendorFollower;

    private SmartMotorController flywheelMotor;
    private SmartMotorControllerConfig flywheelMotorConfig  = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(50, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // Gearing from the motor rotor to final shaft.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
        // Motor properties to prevent over currenting.
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25))
    ;

    private FlyWheelConfig flywheelConfig;
    private FlyWheel flywheel;

    private Command dbg_speed_0;
    private Command dbg_speed_1;
    private Command dbg_speed_2;

    public DoubleFlywheel(int flywheelIndex, int id0, int id1, boolean inversion) {
        this.flywheelIndex = flywheelIndex;

        vendorLead = new SparkMax(id0, MotorType.kBrushless);
        vendorFollower = new SparkMax(id1, MotorType.kBrushless);

        flywheelMotor = new SparkWrapper(vendorLead, GEARBOX, 
            flywheelMotorConfig
                .withFollowers(Pair.of(vendorFollower, false))
                .withMotorInverted(inversion)
                .withTelemetry(Constants.Shooter.telemetryYAMSFlywheelList[this.flywheelIndex]+"Motor", Constants.getAppropriateTelemetryLevel())
        );

        flywheelConfig = new FlyWheelConfig(flywheelMotor)
            .withTelemetry(Constants.Shooter.telemetryYAMSFlywheelList[this.flywheelIndex]+"Mech", Constants.getAppropriateTelemetryLevel())
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(1))
            .withUpperSoftLimit(RPM.of(6000))
        ;
        flywheel = new FlyWheel(flywheelConfig);

        dbg_speed_0 = flywheel.runTo(RPM.of(0), RPM.of(6000)).andThen(flywheel.set(0)).withName("dbg_speed_0");
        dbg_speed_1 = flywheel.run(RPM.of(2000)).withName("dbg_speed_1");
        dbg_speed_2 = flywheel.run(RPM.of(4000)).withName("dbg_speed_2");

        Logging.registerDebugCommand(Constants.Shooter.telemetryNamesFlywheel[this.flywheelIndex]+dbg_speed_0.getName(), dbg_speed_0);
        Logging.registerDebugCommand(Constants.Shooter.telemetryNamesFlywheel[this.flywheelIndex]+dbg_speed_1.getName(), dbg_speed_1);
        Logging.registerDebugCommand(Constants.Shooter.telemetryNamesFlywheel[this.flywheelIndex]+dbg_speed_2.getName(), dbg_speed_2);
    }

    @Override
    public void periodic() {
        // flywheel.getMechanismSetpoint().ifPresent(setpoint -> SmartDashboard.putNumber("hoodSetpoint", setpoint.in(RPM)));
        // SmartDashboard.putNumber("hoodSetpoint", flywheel.getSpeed().in(RPM));

        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }
}
