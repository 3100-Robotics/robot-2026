package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final DCMotor GEARBOX = DCMotor.getNEO(2);

    private SparkMax motor0;
    private SparkMax motor1;

    private SparkMaxSim motorsSim;
    private FlywheelSim flywheelModel;

    // private Command spinUp = run(
    //     () -> motor0
    //         .getClosedLoopController()
    //         .setSetpoint(1000, ControlType.kVelocity)
    //     ).withName("spinUp");
    // private Command spinDown = run(() -> motor0.set(0)).withName("spinDown");

    public Flywheel(int id0, int id1) {
        motor0 = new SparkMax(id0, MotorType.kBrushless);
        motor1 = new SparkMax(id1, MotorType.kBrushless);

        motorsSim = new SparkMaxSim(motor0, GEARBOX);
        flywheelModel = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                GEARBOX, 0.0019021577, 1
            ), 
            GEARBOX
        ); // TODO: Fix MOI

        SparkMaxConfig configroot = new SparkMaxConfig();
        configroot
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast)
            .apply(
                new ClosedLoopConfig()
                    .p(0.001)
                    .i(0)
                    .d(0)
                    .outputRange(-1, 1)
                    .apply(
                        new FeedForwardConfig()
                            .kV(0.002)
                    ).feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            )
        ;

        SparkBaseConfig auxConfig = new SparkMaxConfig()
            .apply(configroot)
            .follow(5, true)
        ;

        // Persist parameters to retain configuration in the event of a power cycle
        motor0.configure(configroot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor1.configure(auxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        motor1.resumeFollowerMode();
        // SmartDashboard.putData(spinUp);
        // SmartDashboard.putData(spinDown);
        // SmartDashboard.putBoolean("is the follower following", motor1.isFollower());
        // SmartDashboard.putNumber("Left Out", motor0.getAppliedOutput());
        // SmartDashboard.putNumber("Right Out", motor1.getAppliedOutput());
        // SmartDashboard.putNumber("Angular", flywheelModel.getAngularVelocityRPM());
        motor1.resumeFollowerMode();
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our arm is doing
        // First, we set our "inputs" (voltages)
        
        flywheelModel.setInput(
            motorsSim.getAppliedOutput() * RoboRioSim.getVInVoltage()
        );

        // Next, we update it. The standard loop time is 20ms for some stupid reason.
        flywheelModel.update(0.02);

        // Now, we update the Spark Maxes
        motorsSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
                flywheelModel.getAngularVelocityRadPerSec()),
            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
            0.02); // Time interval, in Seconds

        // SimBattery estimates loaded battery voltages
        // This should include all motors being simulated
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelModel.getCurrentDrawAmps())
        );

        // Update any external GUI displays or values as desired
        // For example, a Mechanism2d Arm based on the simulated arm angle
    }
}
