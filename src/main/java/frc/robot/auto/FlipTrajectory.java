package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Locator;
import frc.robot.math.Direction;

public class FlipTrajectory {
    public static AutoTrajectory flipConditional(Direction side, AutoRoutine routine, AutoTrajectory inputTrajectory) {
        var rawTraj = inputTrajectory.getRawTrajectory();

        List<SwerveSample> sampleList = new ArrayList<>();

        for (var swerveSampleRaw : rawTraj.samples()) {
            SwerveSample sm = (SwerveSample)swerveSampleRaw;
            SwerveSample new_sample = new SwerveSample(
                sm.t,
                sm.x,
                Constants.fieldWidth.in(Meters) - sm.y,
                -sm.heading,
                sm.vx,
                -sm.vy,
                -sm.omega,
                sm.ax,
                -sm.ay,
                -sm.alpha, 
                new double[] {
                    sm.moduleForcesX()[1],
                    sm.moduleForcesX()[0],
                    sm.moduleForcesX()[3],
                    sm.moduleForcesX()[2]
                },
                new double[] {
                    -sm.moduleForcesY()[1],
                    -sm.moduleForcesY()[0],
                    -sm.moduleForcesY()[3],
                    -sm.moduleForcesY()[2]
                }
            );
            sampleList.add(new_sample);
        }

        Trajectory<SwerveSample> new_trajectory = new Trajectory<SwerveSample>(
            rawTraj.name(),
            sampleList,
            rawTraj.splits(),
            rawTraj.events()
        );

        if (side==Direction.Right) {
            return inputTrajectory;
        } else {
            return routine.trajectory(new_trajectory);
        }
    }

    // public static Trajectory flip(Trajectory inputTrajectory) {
    //     var outputTrajectory = new Trajectory();
    //     List<State> stateList = new ArrayList<>();
    //     for (State state : inputTrajectory.getStates()) {
    //         var newState = new State();
    //         newState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
    //         newState.curvatureRadPerMeter = state.curvatureRadPerMeter;
    //         newState.timeSeconds = state.timeSeconds;
    //         newState.velocityMetersPerSecond = state.velocityMetersPerSecond;

    //         newState.poseMeters = new Pose2d(
    //             state.poseMeters.getX(),
    //             (2*(Constants.fieldWidth.in(Meters)/2))-state.poseMeters.getY(),
    //             state.poseMeters.getRotation().unaryMinus()
    //         );

    //         stateList.add(newState);
    //     }
    //     outputTrajectory.concatenate(new Trajectory(stateList));
    //     return outputTrajectory;
    // }

    // public static Trajectory flip2(Trajectory inputTrajectory) {
    //     var outputTrajectory = new Trajectory();
    //     List<State> stateList = new ArrayList<>();
    //     for (State state : inputTrajectory.getStates()) {
    //         var newState = new State();
    //         newState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
    //         newState.curvatureRadPerMeter = state.curvatureRadPerMeter;
    //         newState.timeSeconds = state.timeSeconds;
    //         newState.velocityMetersPerSecond = state.velocityMetersPerSecond;

    //         newState.poseMeters = new Pose2d(
    //             state.poseMeters.getX(),
    //             (2*(Constants.fieldWidth.in(Meters)/2))-state.poseMeters.getY(),
    //             state.poseMeters.getRotation().unaryMinus()
    //         );

    //         stateList.add(newState);
    //     }
    //     outputTrajectory.concatenate(new Trajectory(stateList));
    //     return outputTrajectory;
    // }
}
