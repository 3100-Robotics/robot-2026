package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Supplier;

public class ShooterFlywheel extends SubsystemBase {
    private final DCMotor GEARBOX = DCMotor.getNEO(1);

    private final int flywheelIndex;

    private SparkMax vendorLead;

    public Supplier<AngularVelocity> speedTargetProvider = () -> RPM.of(0);

    public ShooterFlywheel(
            int flywheelIndex, int id0, int id1, boolean inversion, double kp, double kv) {
        this.flywheelIndex = flywheelIndex;

        vendorLead = new SparkMax(id0, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
                Constants.Shooter.YAMS.flywheelNames[this.flywheelIndex] + "Current",
                vendorLead.getOutputCurrent());
    }

    @Override
    public void simulationPeriodic() {}
}
