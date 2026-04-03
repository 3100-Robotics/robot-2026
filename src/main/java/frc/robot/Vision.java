package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public BooleanSubscriber leftHasTarget = NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/Left/hasTarget").subscribe(false);
    public BooleanSubscriber rightHasTarget = NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/Right/hasTarget").subscribe(false);

    public Field2d purevision = new Field2d();
    public StructPublisher<Pose3d> publish3d0 = NetworkTableInstance.getDefault()
        .getStructTopic("03d", Pose3d.struct).publish();
    public StructPublisher<Pose3d> rightcam3d = NetworkTableInstance.getDefault()
        .getStructTopic("lcam3d", Pose3d.struct).publish();

    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    public boolean usePose = true;
    private AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final Transform3d robotToFrontRight =
        new Transform3d(
            new Translation3d(
                Inches.of(-7.822376).in(Meters), 
                Inches.of(-10.446815).in(Meters),
                Inches.of(28.002807).in(Meters)
            ),
            new Rotation3d(0, Math.toRadians(-25), Math.toRadians(0))
        );

    public static final Transform3d robotToFrontLeft =
        new Transform3d(
            new Translation3d(
                Inches.of(-7.128678).in(Meters), 
                Inches.of(10.086332).in(Meters),
                Inches.of(28.077753).in(Meters)
            ),
            // new Rotation3d(0, Math.toRadians(-25), Math.toRadians(0))
            //     .rotateBy(new Rotation3d(0, 0, -20))
            new Rotation3d(0, Math.toRadians(-25), Math.toRadians(0))
                .rotateBy(new Rotation3d(0, 0, Math.toRadians(-20)))
        );

    public PhotonPoseEstimator photonEstimatorFrontRight;
    public PhotonPoseEstimator photonEstimatorFrontLeft;
    public PhotonCamera cameraFrontRight = new PhotonCamera("Right");
    public PhotonCamera cameraFrontLeft = new PhotonCamera("Left");

    // Simulation

    // This really is just for sim, vision has nothing to do with it
    public Supplier<Pose2d> robotPoseFromDrivetrain;

    public VisionSystemSim visionSim;

    public PhotonCameraSim cameraSimFrontRight;
    public PhotonCameraSim cameraSimFrontLeft;

    public Vision(EstimateConsumer estConsumer, Supplier<Pose2d> robotPoseFromDrivetrain) {
        this.robotPoseFromDrivetrain = robotPoseFromDrivetrain;
        photonEstimatorFrontRight = new PhotonPoseEstimator(
            tagLayout,
            robotToFrontRight
        );

        photonEstimatorFrontLeft = new PhotonPoseEstimator(
            tagLayout,
            robotToFrontLeft);

        this.estConsumer = estConsumer;
        if (!Robot.isReal()) {
            // `VisionSim` shows up in networktables
            visionSim = new VisionSystemSim("VisionSim");
            visionSim.addAprilTags(tagLayout);

            SimCameraProperties cameraPropFrontRight = new SimCameraProperties();
            SimCameraProperties cameraPropFrontLeft = new SimCameraProperties();

            cameraPropFrontRight.setCalibration(1280, 720, Rotation2d.fromDegrees(72));
            cameraPropFrontRight.setCalibError(0.25, 0.08);
            cameraPropFrontRight.setFPS(20);
            cameraPropFrontRight.setAvgLatencyMs(35);
            cameraPropFrontRight.setLatencyStdDevMs(5);

            cameraPropFrontLeft.setCalibration(1280, 720, Rotation2d.fromDegrees(72));
            cameraPropFrontLeft.setCalibError(0.25, 0.08);
            cameraPropFrontLeft.setFPS(20);
            cameraPropFrontLeft.setAvgLatencyMs(35);
            cameraPropFrontLeft.setLatencyStdDevMs(5);

            cameraSimFrontRight = new PhotonCameraSim(cameraFrontRight, cameraPropFrontRight);
            cameraSimFrontLeft = new PhotonCameraSim(cameraFrontLeft, cameraPropFrontLeft);

            cameraSimFrontLeft.enableDrawWireframe(true);
            cameraSimFrontRight.enableDrawWireframe(true);

            visionSim.addCamera(cameraSimFrontRight, robotToFrontRight);
            visionSim.addCamera(cameraSimFrontLeft, robotToFrontLeft);
        }
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(robotPoseFromDrivetrain.get());
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("usingPose", usePose);
        SmartDashboard.putData("purevision", purevision);
        rightcam3d.set(new Pose3d(robotToFrontRight.getTranslation(), robotToFrontRight.getRotation()));

        // if (
        //     !leftHasTarget.get() &&
        //     rightHasTarget.get()
        // ) {

            Optional<EstimatedRobotPose> visionEstRight = Optional.empty();
            for (var result : cameraFrontRight.getAllUnreadResults()) {
                visionEstRight = photonEstimatorFrontRight.estimateCoprocMultiTagPose(result);
                if (visionEstRight.isEmpty()) {
                    visionEstRight = photonEstimatorFrontRight.estimateLowestAmbiguityPose(result);
                }
                updateEstimationStdDevs(visionEstRight, result.getTargets());

                if (Robot.isSimulation()) {
                    visionEstRight.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        }
                    );
                } else {
                    visionEstRight.ifPresent(
                        est -> {
                            var estStdDevs = getEstimationStdDevs();
                            purevision.setRobotPose(est.estimatedPose.toPose2d());
                            if (usePose) {
                                estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                            }
                        }
                    );
                }
            }
        
        // }


        // LEft Camera!

            // Optional<EstimatedRobotPose> visionEstLeft = Optional.empty();
            // for (var result : cameraFrontLeft.getAllUnreadResults()) {
            //     visionEstLeft = photonEstimatorFrontLeft.estimateCoprocMultiTagPose(result);
            //     if (visionEstLeft.isEmpty()) {
            //         visionEstLeft = photonEstimatorFrontLeft.estimateLowestAmbiguityPose(result);
            //     }
            //     updateEstimationStdDevs(visionEstLeft, result.getTargets());

            //     if (Robot.isSimulation()) {
            //         visionEstLeft.ifPresentOrElse(
            //             est ->
            //                     getSimDebugField()
            //                             .getObject("VisionEstimation")
            //                             .setPose(est.estimatedPose.toPose2d()),
            //             () -> {
            //                 getSimDebugField().getObject("VisionEstimation").setPoses();
            //             }
            //         );
            //     } else {
            //         visionEstLeft.ifPresent(
            //             est -> {
            //                 var estStdDevs = getEstimationStdDevs();
            //                 // purevision.setRobotPose(est.estimatedPose.toPose2d());
            //                 if (usePose) {
            //                     estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            //                 }
            //             }
            //         );
            //     }
            // }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, 
            List<PhotonTrackedTarget> targets
    ) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimatorFrontRight.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}