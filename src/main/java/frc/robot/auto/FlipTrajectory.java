package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.utils.Direction;

public class FlipTrajectory {
    public static AutoTrajectory flipConditional(Direction side, AutoRoutine routine, AutoTrajectory inputTrajectory) {
        if (side==Direction.Right) {
            return inputTrajectory;
        } else {
            return inputTrajectory.mirrorY();
        }
    }
}
