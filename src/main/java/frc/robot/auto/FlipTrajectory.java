package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.Constants;
import frc.robot.Locator;

public class FlipTrajectory {
    public static Trajectory flip(Trajectory inputTrajectory) {
        var outputTrajectory = new Trajectory();
        List<State> stateList = new ArrayList<>();
        for (State state : inputTrajectory.getStates()) {
            var newState = new State();
            newState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
            newState.curvatureRadPerMeter = state.curvatureRadPerMeter;
            newState.timeSeconds = state.timeSeconds;
            newState.velocityMetersPerSecond = state.velocityMetersPerSecond;

            newState.poseMeters = new Pose2d(
                state.poseMeters.getX(),
                (2*(Constants.fieldWidth.in(Meters)/2))-state.poseMeters.getY(),
                state.poseMeters.getRotation().unaryMinus()
            );

            stateList.add(newState);
        }
        outputTrajectory.concatenate(new Trajectory(stateList));
        return outputTrajectory;
    }
}
