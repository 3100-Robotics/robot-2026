package frc.robot.utils;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public enum PivotState {
    Deployed(Constants.Intake.pivotDeployAngle), Stow(Constants.Intake.pivotStowAngle), SuperStow(Constants.Intake.pivotSuperStowAngle);

    public Angle angle; 

    PivotState(Angle angle) {
        this.angle = angle;
    }
}