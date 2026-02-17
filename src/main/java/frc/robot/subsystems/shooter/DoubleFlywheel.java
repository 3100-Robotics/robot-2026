package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularMomentum;
import edu.wpi.first.units.measure.AngularVelocity;
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

public class DoubleFlywheel extends SubsystemBase {
    private final DCMotor GEARBOX = DCMotor.getNEO(2);

    private final int flywheelIndex;

    private SparkMax vendorLead;
    private SparkMax vendorFollower;

    private SmartMotorController flywheelMotor;
    private SmartMotorControllerConfig flywheelMotorConfig  = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // Feedback Constants (PID Constants)
        .withClosedLoopController(0.0098, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(50, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // Feedforward Constants
        .withFeedforward(new SimpleMotorFeedforward(0, 0.128, 0))
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
    public FlyWheel flywheel;

    private Command dbg_speed_0;
    private Command dbg_speed_1;
    private Command dbg_speed_2;

    public Supplier<AngularVelocity> speedTargetProvider = () -> RPM.of(0);

    public DoubleFlywheel(int flywheelIndex, int id0, int id1, boolean inversion) {
        this.flywheelIndex = flywheelIndex;

        vendorLead = new SparkMax(id0, MotorType.kBrushless);
        vendorFollower = new SparkMax(id1, MotorType.kBrushless);

        flywheelMotor = new SparkWrapper(vendorLead, GEARBOX, 
            flywheelMotorConfig
                .withFollowers(Pair.of(vendorFollower, false))
                .withMotorInverted(inversion)
                .withTelemetry(Constants.Shooter.YAMS.flywheelNames[this.flywheelIndex]+"Motor", Constants.getAppropriateTelemetryLevel())
        );

        flywheelConfig = new FlyWheelConfig(flywheelMotor)
            .withTelemetry(Constants.Shooter.YAMS.flywheelNames[this.flywheelIndex]+"Mech", Constants.getAppropriateTelemetryLevel())
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(1))
            .withUpperSoftLimit(RPM.of(6000))
        ;
        flywheel = new FlyWheel(flywheelConfig);

        // If it actually trys to run at 0 it will jitter, so its easier to juts set duty cycle 0
        // after using a control loop to 'set it to 0' with massive room for error, making
        // the first half of the command terminate instantly
        dbg_speed_0 = flywheel.runTo(RPM.of(0), RPM.of(6000)).andThen(flywheel.set(0))
            .withName("dbg_speed_0");
        dbg_speed_1 = flywheel.run(RPM.of(2000))
            .withName("dbg_speed_1");
        dbg_speed_2 = flywheel.run(RPM.of(4000))
            .withName("dbg_speed_2");

        Logging.registerDebugCommand(
            Constants.Shooter.Main.flywheelNames[this.flywheelIndex]+dbg_speed_0.getName(), dbg_speed_0);
        Logging.registerDebugCommand(
            Constants.Shooter.Main.flywheelNames[this.flywheelIndex]+dbg_speed_1.getName(), dbg_speed_1);
        Logging.registerDebugCommand(
            Constants.Shooter.Main.flywheelNames[this.flywheelIndex]+dbg_speed_2.getName(), dbg_speed_2);

        // stop = flywheel.runTo(RPM.of(0), RPM.of(6000)).andThen(flywheel.set(0));
        // runAtCurrentTarget = flywheel.run(speedTarget);
    }

    public void setTarget(Supplier<AngularVelocity> newSpeedTarget) {
        speedTargetProvider = newSpeedTarget;
    }

    public Command runNewTarget(Supplier<AngularVelocity> newSpeedTarget) {
        return runOnce(() -> speedTargetProvider = newSpeedTarget)
                .andThen(runAtCurrentTarget());
    }

    public Command runAtCurrentTarget() {
        return flywheel.run(speedTargetProvider);
    }

    public Command runAtCurrentTarget2() {
        return Commands.parallel(
            flywheel.setSpeed(speedTargetProvider)//,
            // Commands.run(() -> SmartDashboard.putNumber("Number two three four", speedTargetProvider.get().in(RPM)))//,
            // Commands.run(() -> System.out.println(flywheel))
        );
    }

    public Command stop() {
        return flywheel.runTo(RPM.of(0), RPM.of(6000))
            .andThen(flywheel.set(0));
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
