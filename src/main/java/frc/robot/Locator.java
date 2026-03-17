package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Locator extends SubsystemBase {
    public static Locator instance;

    public boolean hasAppliedAlliance = false;
    public Optional<Alliance> alliance = Optional.empty();

    public Pose2d hubPose = Constants.hubPoseBlue;

    private final Field2d field = new Field2d();

    private Pose2d robotPose = new Pose2d();
    private Supplier<Pose2d> getRobotPose = () -> new Pose2d();
    public final Supplier<Distance> distanceToHub = () -> {
        double distance = Math.sqrt(
            Math.pow(robotPose.getMeasureX().minus(hubPose.getMeasureX()).in(Inches), 2) +
            Math.pow(robotPose.getMeasureY().minus(hubPose.getMeasureY()).in(Inches), 2)
        );
        return Inches.of(distance);
    };

    public Pose2d extentionPose = new Pose2d();

    public Locator(Supplier<Pose2d> getRobotPose) {
        if (instance == null) {
            instance = this;
        }
        this.getRobotPose = getRobotPose;
    }

    public static Locator getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("best field ever", field);
        SmartDashboard.putBoolean("recvAllianceColor", hasAppliedAlliance);
        SmartDashboard.putNumber("robotDistanceToHubFeet", distanceToHub.get().in(Feet));

        if (!hasAppliedAlliance || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                alliance = Optional.of(allianceColor);
                hasAppliedAlliance = true;
            });

            alliance.ifPresent(
                allianceColor -> hubPose = allianceColor == Alliance.Red
                    ? Constants.hubPoseRed :
                    Constants.hubPoseBlue
            );
            SmartDashboard.putString("lookatme", alliance.toString());
            field.getObject("hubyentoo").setPose(hubPose);
        }

        robotPose = getRobotPose.get();
        FieldObject2d targetHub = field.getObject("Hub");
        FieldObject2d targetExtension = field.getObject("Extension");

        this.extentionPose = new Pose2d(
            3.36*Math.cos(targetHub.getPose().getRotation().getRadians())+hubPose.getX(),
            3.36*Math.sin(targetHub.getPose().getRotation().getRadians())+hubPose.getY(),
            targetHub.getPose().getRotation().rotateBy(Rotation2d.k180deg)
        );

        field.setRobotPose(robotPose);
        targetHub.setPose(
            new Pose2d(
                hubPose.getX(),
                hubPose.getY(),
                new Rotation2d(Math.PI+Math.atan2(hubPose.getY()-robotPose.getY(), hubPose.getX()-robotPose.getX()))
            )
        );
        targetExtension.setPose(extentionPose);
    }
}
