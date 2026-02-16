package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class Roller extends SubsystemBase {
    private SparkMax vendorMotor;
    private SmartMotorController motor;
    private SmartMotorControllerConfig motorConfig;
    private FlyWheelConfig rollerConfig;
    private FlyWheel rollerMech;

    public Roller(String name, int motorId, SmartMotorControllerConfig motorConfig, FlyWheelConfig rollerConfig) {
        vendorMotor = new SparkMax(motorId, MotorType.kBrushless);
        this.motorConfig = motorConfig.clone()
            .withSubsystem(this)
            .withTelemetry(Constants.Indexer.telemetryYAMSName+name+"_Motor", Constants.getAppropriateTelemetryLevel())
            ;
        
        motor = new SparkWrapper(vendorMotor, DCMotor.getNEO(1), this.motorConfig);

        this.rollerConfig = rollerConfig.clone()
            .withSmartMotorController(motor)
            .withTelemetry(Constants.Indexer.telemetryYAMSName+name+"_Mech", Constants.getAppropriateTelemetryLevel())
            ;
        rollerMech = new FlyWheel(this.rollerConfig);
    }

    public Command stop() {
        return rollerMech.setSpeed(RPM.of(0))
            .withTimeout(Second.of(1))
            .andThen(rollerMech.set(0));
    }

    public Command runAtSpeed(AngularVelocity speed) {
        return rollerMech.setSpeed(speed);
    }
}
