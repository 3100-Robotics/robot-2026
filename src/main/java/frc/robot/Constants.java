package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.math.SpeedSet;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Constants {
    public static final boolean doLiveTuning = false;

    public static final boolean enableShooter = true;
    public static final boolean enableIndexer = true;
    public static final boolean enableIntake = true;
    public static final boolean enableDrivetrain = true;

    public static final Distance fieldLength = Inches.of(651.22);
    public static final Distance fieldWidth = Inches.of(317.69);

    public static final Pose2d hubPoseBlue = new Pose2d(4.628518104553223, 4.035704612731934, new Rotation2d());
    public static final Pose2d hubPoseRed = new Pose2d(4.628518104553223, 4.035704612731934, new Rotation2d())
        .rotateAround(new Translation2d(
            fieldLength.div(2),
            fieldWidth.div(2)
        ), Rotation2d.fromDegrees(180));

    public static TelemetryVerbosity getAppropriateTelemetryLevel() {
        return doLiveTuning ? TelemetryVerbosity.HIGH : TelemetryVerbosity.LOW;
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
        public static Angle maxHoodAngle = Degrees.of(40);
        public static Angle minHoodAngle = Degrees.of(12.667292);

        public static class Physical {
            public static final List<Pair<Distance, Double>> distanceAngleTable = new ArrayList<>();
            public static final List<Pair<Distance, Double>> distanceSpeedTable = new ArrayList<>();

            static {
                // Distance Angle
                distanceAngleTable.add(Pair.of(Meters.of(1.36), 20.0-2.5));
                distanceAngleTable.add(Pair.of(Meters.of(5.86), 40.0-2.5));

                // Distance Speed
                distanceSpeedTable.add(Pair.of(Meters.of(1.36), 2400.0+(2400*0.05)));
                distanceSpeedTable.add(Pair.of(Meters.of(5.86), 3960.0+(3960*0.05)));
               
            }
        }

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
    }

    public static class Indexer {
        public static final SpeedSet off = new SpeedSet(
            RPM.of(0),
            RPM.of(0),
            RPM.of(0)
        );

        public static final SpeedSet on = new SpeedSet(
            RPM.of(4414),
            RPM.of(4414),
            RPM.of(2377)
        );

        public static final double kickerRatio = 36.0/24.0;
        public static final double ceilingRatio = 20.0/24.0;
        public static final double floorRatio = 20.0/24.0;

        public static final double kickerRatioRecip = 1.0 / kickerRatio;
        public static final double ceilingRatioRecip = 1.0 / ceilingRatio;
        public static final double floorRatioRecip = 1.0 / floorRatio;

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
        public static final double rollerRatio = 3.0/2.0;
        public static final double rollerRatioRecip = 1.0 / rollerRatio;

        // Physical Constants
        public static final Angle pivotDeployAngle = Degrees.of(0);
        public static final Angle pivotStowAngle = Degrees.of(-55);

        // Software Constants
        public static final int pivotMotorID = 30;
        public static final int rollerMotorID = 31;
        public static final int encoderID = 32;

        public static final String telemetryNameRoot = "Intake/";
        public static final String telemetryNamePivot = telemetryNameRoot+"Pivot/";
        public static final String telemetryNameRoller = telemetryNameRoot+"Roller/";

        public static final String telemetryYAMSPivot = "Pivot_";
        public static final String telemetryYAMSRoller = "Roller_";
    }

    public static class Vision {
        
    }
}
