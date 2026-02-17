package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    /*
     * Okay, so Okay :}
     * 
     * We split the functions of the shooter into
     * two flywheels and a hood because of how YAMS
     * functions
     */

    private Angle angle = Degrees.of(0);
    private Supplier<Angle> angleProvider = () -> angle;

    private final Hood hood = new Hood(angleProvider);
    private final DoubleFlywheel flywheelL = new DoubleFlywheel(0, 51, 52, false);
    private final DoubleFlywheel flywheelR = new DoubleFlywheel(1, 53, 54, true);

    public Command idle = Commands.parallel(hood.stop, flywheelL.stop, flywheelR.stop);
    public Command stopFlywheels = Commands.parallel(flywheelL.stop, flywheelR.stop);

    public void setAngle(Angle newAngle) {
        if (newAngle.in(Degrees) > Constants.Shooter.maxHoodAngle.in(Degrees)) {
            angle = Constants.Shooter.maxHoodAngle;
        } else if (newAngle.in(Degrees) < Constants.Shooter.minHoodAngle.in(Degrees)) {
            angle = Constants.Shooter.minHoodAngle;
        } else {
            angle = newAngle;
        }
    }

    public Angle getAngle() {
        return angle;
    }
}
