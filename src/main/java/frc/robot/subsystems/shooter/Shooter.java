package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
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
    private Supplier<Angle> angleProvider = () -> hoodAngle;

    private final Hood hood = new Hood(angleProvider);
    private final DoubleFlywheel flywheelL = new DoubleFlywheel(0, 51, 52, false);
    private final DoubleFlywheel flywheelR = new DoubleFlywheel(1, 53, 54, true);

    public Command idle = Commands.parallel(hood.stop, flywheelL.stop, flywheelR.stop);
    // public Command stopFlywheels = Commands.parallel(flywheelL.stop, flywheelR.stop);
    private Command spinUpFlywheels = Commands.parallel(flywheelL.runAtCurrentTarget, flywheelR.runAtCurrentTarget);

    public Shooter() {
        Logging.registerDebugCommand(
            Constants.join('/', Constants.Shooter.nameRoot, "idle"), 
            idle
        );
        Logging.registerDebugCommand(
            Constants.join('/', Constants.Shooter.nameRoot, "spinUpFlywheels"), 
            spinUpFlywheels
        );

        Logging.registerDebugValue("test5", int.class);
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
        SmartDashboard.putNumber("test4", (double)Logging.getLTInstance().debugValues.get("test5"));
    }
}
