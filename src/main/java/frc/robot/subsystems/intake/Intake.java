package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Bird == Flipper == Translation
 */
public class Intake extends SubsystemBase {
    private final Pivot pivotSub = new Pivot();
    private final Roller rollerSub = new Roller();

    public Intake() {
        setName("Intake");

        pivotSub.setDefaultCommand(deploy());
        rollerSub.setDefaultCommand(runAtSpeed(RPM.of(-50)));
    }

    public Command stow() {
        return pivotSub.pivot.setAngle(Degrees.of(90));
    }

    public Command deploy() {
        return pivotSub.pivot.setAngle(Degrees.of(4));
    }

    public Command stop() {
        return rollerSub.roller.set(0);
    }

    public Command runAtSpeed(AngularVelocity speed) {
        return rollerSub.roller.setSpeed(speed);
    }
}
