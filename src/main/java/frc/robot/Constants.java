package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Constants {
    public static final boolean doLiveTuning = true;

    public static final boolean enableShooter = true;
    public static final boolean enableIndexer = true;
    public static final boolean enableIntake = false;
    public static final boolean enableDrivetrain = false;

    public static final Distance fieldLength = Inches.of(651.22);
    public static final Distance fieldWidth = Inches.of(317.69);

    public static final Pose2d hubPoseBlue = new Pose2d(4.628518104553223, 4.035704612731934, new Rotation2d());
    public static final Pose2d hubPoseRed = new Pose2d(4.628518104553223, 4.035704612731934, new Rotation2d())
        .rotateAround(new Translation2d(
            fieldLength.div(2),
            fieldWidth.div(2)
        ), Rotation2d.fromDegrees(180));

    public static TelemetryVerbosity getAppropriateTelemetryLevel() {
        return doLiveTuning ? TelemetryVerbosity.HIGH : TelemetryVerbosity.MID;
    }

    public static String join(char delimeter, String... strings) {
        List<String> list = Arrays.asList(strings);
        return String.join(String.valueOf(delimeter), list);
    }

    public static String joinYAMS(String... strings) {
        List<String> list = Arrays.asList(strings);
        return String.join("_", list);
    }

    public static class Shooter {
        /*
         * Flywheel L IS the left flywheel IS flywheel 0
         * Flywheel R IS the right flywheel IS flywheel 1
         */
        public static Angle maxHoodAngle = Degrees.of(50);
        public static Angle minHoodAngle = Degrees.of(0);

        public static int hoodMotorID = 50;
        public static int flywheel0MotorID = 51;
        public static int flywheel1MotorID = 52;
        public static int flywheel2MotorID = 53;
        public static int flywheel3MotorID = 54;

        public static String nameRoot = "Shooter";

        public static class Main {
            public static String nameFlywheelL = join('/', nameRoot, "Flywheel_L");
            public static String nameFlywheelR = join('/', nameRoot, "Flywheel_R");
            public static String flywheelNames[] = {nameFlywheelL, nameFlywheelR};
            public static String nameHood = "Hood";
        }

        public static class YAMS {
            public static String nameFlywheelL = joinYAMS(nameRoot, "FlywheelL_");
            public static String nameFlywheelR = joinYAMS(nameRoot, "FlywheelR_");
            public static String flywheelNames[] = {nameFlywheelL, nameFlywheelR};
            public static String nameHood = joinYAMS(nameRoot, "Hood");
        }

        // public static String telemetryNameFlywheelL = telemetryNameRoot+"Flywheel_L/";
        // public static String telemetryNameFlywheelR = telemetryNameRoot+"Flywheel_R/";
        // public static String telemetryNamesFlywheel[] = {telemetryNameFlywheelL, telemetryNameFlywheelR};
        // public static String telemetryNameHood = telemetryNameRoot+"Hood/";

        // public static String telemetryYAMSFlywheelL = "FlywheelL_";
        // public static String telemetryYAMSFlywheelR = "FlywheelR_";
        // public static String telemetryYAMSFlywheelList[] = {telemetryYAMSFlywheelL, telemetryYAMSFlywheelR};
    }

    public static class Indexer {
        public static int floorMotorID = 40;
        public static int ceilingMotorID = 41;
        public static int kickerMotorID = 42;

        public static String nameRoot = "Indexer";

        public static class Main {
            public static String nameFloor = join('/', nameRoot, "Floor");
            public static String nameCeiling = join('/', nameRoot, "Ceiling");
            public static String nameKicker = join('/', nameRoot, "Kicker");
        }

        public static class YAMS {
            public static HashMap<String, String> internal = new HashMap<>();

            public static String nameFloor = joinYAMS(nameRoot, "Floor");
            public static String nameCeiling = joinYAMS(nameRoot, "Ceiling");
            public static String nameKicker = joinYAMS(nameRoot, "Kicker");

            static {
                internal.put(Main.nameFloor, nameFloor);
                internal.put(Main.nameCeiling, nameCeiling);
                internal.put(Main.nameKicker, nameKicker);
            }

            public static String get(String key) {
                return internal.get(key);
            }
        }
    }

    public static class Intake {
        public static int pivotMotorID = 30;
        public static int rollerMotorID = 31;
        public static int encoderID = 32;

        public static String telemetryNameRoot = "Intake/";
        public static String telemetryNamePivot = telemetryNameRoot+"Pivot/";
        public static String telemetryNameRoller = telemetryNameRoot+"Roller/";

        public static String telemetryYAMSPivot = "Pivot_";
        public static String telemetryYAMSRoller = "Roller_";
    }

    public static class Vision {

    }
}
