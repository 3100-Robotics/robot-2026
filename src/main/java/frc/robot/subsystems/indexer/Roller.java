package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.RPM;
import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Roller extends SubsystemBase {
    private Optional<SparkMax> vendorMotorSparkMax;
    private Optional<TalonFX> vendorMotorTalonFX;
    public SmartMotorController motor;
    private SmartMotorControllerConfig motorConfig;
    // private FlyWheelConfig rollerConfig;
    // private FlyWheel rollerMech;

    private void CommonSetup(String name, DCMotor motors, SmartMotorControllerConfig motorConfig) {
        setName(Constants.Indexer.YAMS.get(name));
        this.motorConfig = motorConfig.clone()
            .withSubsystem(this)
            .withTelemetry(Constants.Indexer.YAMS.get(name)+"_Motor", Constants.getAppropriateTelemetryLevel())
        ;

        vendorMotorSparkMax.ifPresent(
            vendorSparkMax -> this.motor = new SparkWrapper(vendorSparkMax, motors, this.motorConfig)
        );

        vendorMotorTalonFX.ifPresent(
            vendorTalonFX -> this.motor = new TalonFXWrapper(vendorTalonFX, motors, this.motorConfig)
        );

        // if (vendorMotorTalonFX.isPresent()) {
        //     this.motor = new TalonFXWrapper(vendorMotorTalonFX.orElseThrow(), motors, this.motorConfig);
        // } else if (vendorMotorSparkMax.isPresent()) {
        //     this.motor = new SparkWrapper(vendorMotorSparkMax.orElseThrow(), motors, this.motorConfig);
        // }
        
        // rollerMech = new FlyWheel(this.rollerConfig);
    }

    public Roller(String name, DCMotor motors, SparkMax vendorMotor, SmartMotorControllerConfig motorConfig) {
        this.vendorMotorSparkMax = Optional.of(vendorMotor);
        this.vendorMotorTalonFX = Optional.empty();
        CommonSetup(name, motors, motorConfig);
    }

    public Roller(String name, DCMotor motors, TalonFX vendorMotor, SmartMotorControllerConfig motorConfig) {
        this.vendorMotorTalonFX = Optional.of(vendorMotor);
        this.vendorMotorSparkMax = Optional.empty();
        CommonSetup(name, motors, motorConfig);
    }

    // public Command stop() {
    //     return motor.runTo(RPM.of(0), RPM.of(6000))
    //         .andThen(rollerMech.set(0));
    // }

    // public Command runAtSpeed(AngularVelocity speed) {
    //     return rollerMech.setSpeed(speed);
    // }


    public Command stop() {
        return runOnce(motor::startClosedLoopController)
            .andThen(runOnce(() -> motor.setVelocity(RPM.of(0))))
            .andThen(Commands.waitUntil(
                new Trigger(() -> motor.getMechanismVelocity().isNear(RPM.of(0), RPM.of(6000)))
                    .debounce(0.1, DebounceType.kRising)
            ))
            .andThen(run(() -> motor.setDutyCycle(0)));
    }

    public Command runAtSpeed(AngularVelocity speed) {
        return run(() -> motor.setVelocity(speed));
    }

    @Override
    public void periodic() {
        motor.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        motor.simIterate();
    }
}
