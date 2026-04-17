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
import frc.robot.subsystems.Drivetrain;

public class Locator {
    public static Locator instance;
    public Drivetrain drivetrain;

    public boolean hasAppliedAlliance = false;
    public Optional<Alliance> alliance = Optional.empty();

    public Pose2d hubPose = Constants.hubPoseBlue;

    private final Field2d field = new Field2d();

    // private Pose2d robotPose = new Pose2d();
    // private Supplier<Pose2d> getRobotPose = () -> new Pose2d();
    // public final Supplier<Distance> distanceToHub = () -> {
    //     double distance = Math.sqrt(
    //         Math.pow(robotPose.getMeasureX().minus(hubPose.getMeasureX()).in(Inches), 2) + 
    //         Math.pow(robotPose.getMeasureY().minus(hubPose.getMeasureY()).in(Inches), 2)
    //     );
    //     return Inches.of(distance);
    // };

    // public Pose2d extentionPose = new Pose2d();

    public Locator(Supplier<Pose2d> getRobotPose, Drivetrain drivetrain) {
        Robot.getInstance().addPeriodic(this::periodicAllianceApplied, 1);
        Robot.getInstance().addPeriodic(this::periodicSmartDashboard, 0.1);
        if (instance == null) {
            instance = this;
        }
        // this.getRobotPose = getRobotPose;
        this.drivetrain = drivetrain;
    }

    public static Locator getInstance() {
        return instance;
    }

    public Pose2d getRobotPose() {
        return drivetrain.getPos();
    }

    public Pose2d getExtensionPose() {
        var thub = new Rotation2d(Math.PI+Math.atan2(hubPose.getY()-getRobotPose().getY(), hubPose.getX()-getRobotPose().getX()));
        return new Pose2d(
            2.1*Math.cos(thub.getRadians())+hubPose.getX(),
            2.1*Math.sin(thub.getRadians())+hubPose.getY(),
            thub.rotateBy(Rotation2d.k180deg)
        );
    }

    public Distance getDistanceToHub() {
        double distance = Math.sqrt(
            Math.pow(getRobotPose().getMeasureX().minus(hubPose.getMeasureX()).in(Inches), 2) + 
            Math.pow(getRobotPose().getMeasureY().minus(hubPose.getMeasureY()).in(Inches), 2)
        );
        return Inches.of(distance);
    }


    // Periodics

    // Smartdashboard values every 0.1 seconds
    public void periodicSmartDashboard() {
        SmartDashboard.putData("best field ever", field);
        SmartDashboard.putBoolean("recvAllianceColor", hasAppliedAlliance);
        // SmartDashboard.putNumber("robotDistanceToHubFeet", distanceToHub.get().in(Feet));
        field.setRobotPose(getRobotPose());
    }

    // 1 second
    public void periodicAllianceApplied() {
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
    }
}
