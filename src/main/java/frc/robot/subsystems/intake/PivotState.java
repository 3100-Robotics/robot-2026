package frc.robot.subsystems.intake;


import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public enum PivotState {
    stow(Constants.Intake.pivotStowAngle),
    deploy(Constants.Intake.pivotDeployAngle)
    ;

    public final Angle angle;

    PivotState(Angle angle) {
        this.angle = angle;
    }
}
