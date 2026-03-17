package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public enum RollerState {
    on(Constants.Intake.rollerSpeedNominal),
    off(RPM.of(0))
    ;

    public final AngularVelocity speed;

    RollerState(AngularVelocity speed) {
        this.speed = speed;
    }
}
