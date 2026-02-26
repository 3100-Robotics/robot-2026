package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Locator;

public class AutoConsts {
    public static Pose2d colorPose(Pose2d inputpose) {
        var allianceColor = Locator.getInstance().alliance;
        if (allianceColor.isPresent()) {
            return allianceColor.get() == Alliance.Red ? inputpose.rotateAround(new Translation2d(
                Constants.fieldLength.div(2),
                Constants.fieldWidth.div(2)
            ), Rotation2d.fromDegrees(180)) : inputpose;
        } else {
            return inputpose;
        }
    }

    public static class OPAS {
        public static Pose2d INIT_POSE = new Pose2d(3.560119867324829, 1.6879488229751587, Rotation2d.kCCW_90deg);
        public static Pose2d[] poses = {
            new Pose2d(0.8928170204162598, 0.6429935693740845, Rotation2d.k180deg),
            new Pose2d(2.467463970184326, 2.209561347961426, Rotation2d.kZero)
        };
    }

    public static class LEFT {
        public static Pose2d INIT_POSE = new Pose2d(
            3.6330790519714355,
            6.032428741455078,
            Rotation2d.kCW_90deg
        );

        public static Pose2d[] poses = {
            new Pose2d(1.2054568529129028, 6.877720355987549, Rotation2d.kCW_90deg)
        };
    }
}
