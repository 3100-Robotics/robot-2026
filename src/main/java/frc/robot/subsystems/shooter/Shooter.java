package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularMomentum;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logging;

public class Shooter extends SubsystemBase {
    /*
     * Okay, so Okay :}
     * 
     * We split the functions of the shooter into
     * two flywheels and a hood because of how YAMS
     * functions
     */

    private Angle hoodAngle = Degrees.of(0);
    private AngularVelocity flywheelSpeed = RPM.of(0);

    private Supplier<Angle> angleProvider = () -> hoodAngle;
    private Supplier<AngularVelocity> speedProvider = () -> flywheelSpeed;

    // private final Hood hood = new Hood();
    private final DoubleFlywheel flywheelL = new DoubleFlywheel(0, 51, 52, true);
    private final DoubleFlywheel flywheelR = new DoubleFlywheel(1, 53, 54, false);

    public Shooter() {
        flywheelL.setTarget(
            () -> RPM.of(
                800 // SmartDashboard.getNumber(Constants.join('/', Constants.Shooter.nameRoot, "debugRPM"), 50)
            )
        );

        flywheelR.setTarget(
            () -> RPM.of(
                800 // SmartDashboard.getNumber(Constants.join('/', Constants.Shooter.nameRoot, "debugRPM"), 50)
            )
        );

        Logging.registerDebugValue(
            Constants.join('/', Constants.Shooter.nameRoot, "debugRPM"), double.class
        );

        Logging.registerDebugCommand(
            Constants.join('/', Constants.Shooter.nameRoot, "idle"), 
            // hood.stop().alongWith
            flywheelL.stop().alongWith(flywheelR.stop())
        );

        Logging.registerDebugCommand(
            Constants.join('/', Constants.Shooter.nameRoot, "spinUpFlywheelsToDebugRPM"), 
            flywheelL.runAtCurrentTarget().alongWith(flywheelR.runAtCurrentTarget())
        );
    }

    public void setHoodAngle(Angle newAngle) {
        if (newAngle.in(Degrees) > Constants.Shooter.maxHoodAngle.in(Degrees)) {
            hoodAngle = Constants.Shooter.maxHoodAngle;
        } else if (newAngle.in(Degrees) < Constants.Shooter.minHoodAngle.in(Degrees)) {
            hoodAngle = Constants.Shooter.minHoodAngle;
        } else {
            hoodAngle = newAngle;
        }
    }

    public Angle getHoodAngle() {
        return hoodAngle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("flywheel left rpm", flywheelL.flywheel.getMotor().getMechanismVelocity().in(RPM));
    }
}
