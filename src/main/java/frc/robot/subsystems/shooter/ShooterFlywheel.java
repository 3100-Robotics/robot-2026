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
import edu.wpi.first.math.controller.PIDController;
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

public class ShooterFlywheel extends SubsystemBase {
    private final DCMotor GEARBOX = DCMotor.getNEO(1);

    private final int flywheelIndex;

    private SparkMax vendorLead;

    public Supplier<AngularVelocity> speedTargetProvider = () -> RPM.of(0);

    public ShooterFlywheel(int flywheelIndex, int id0, int id1, boolean inversion, double kp, double kv) {
        this.flywheelIndex = flywheelIndex;

        vendorLead = new SparkMax(id0, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            Constants.Shooter.YAMS.flywheelNames[this.flywheelIndex]+"Current", 
            vendorLead.getOutputCurrent()
        );
    }

    @Override
    public void simulationPeriodic() {
    }
}
