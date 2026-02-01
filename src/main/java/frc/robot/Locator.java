package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Locator extends SubsystemBase {
    private final Pose2d hubPose = new Pose2d(4.628518104553223, 4.035704612731934, new Rotation2d());
    private final Field2d field = new Field2d();

    private Pose2d robotPose = new Pose2d();
    private Supplier<Pose2d> getRobotPose = () -> new Pose2d();
    // private final StructPublisher<Pose2d> chassisPoseNT = Logging.getLTInstance().getRootTable().getStructTopic("ScratchState/Hub", Pose2d.struct).publish();

    public Pose2d extentionPose = new Pose2d();

    public Locator(Supplier<Pose2d> getRobotPose) {
        this.getRobotPose = getRobotPose;
    }

    @Override
    public void periodic() {
        robotPose = getRobotPose.get();
        FieldObject2d targetHub = field.getObject("Hub");
        FieldObject2d targetExtension = field.getObject("Extension");

        this.extentionPose = new Pose2d(
            2*Math.cos(targetHub.getPose().getRotation().getRadians())+hubPose.getX(),
            2*Math.sin(targetHub.getPose().getRotation().getRadians())+hubPose.getY(),
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

        SmartDashboard.putData("ScratchState", field);
        SmartDashboard.putNumber("distance", Math.sqrt(Math.pow(hubPose.getX()-extentionPose.getX(), 2)+Math.pow(hubPose.getY()-extentionPose.getY(), 2)));
    }
}
