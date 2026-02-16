package frc.robot;

import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class Constants {
    public static final boolean doLiveTuning = true;

    public static TelemetryVerbosity getAppropriateTelemetryLevel() {
        return doLiveTuning ? TelemetryVerbosity.HIGH : TelemetryVerbosity.MID;
    }

    public static class Shooter {
        /*
         * Flywheel L IS left flywheel IS flywheel 0
         */
        public static int hoodMotorID = 50;
        public static int flywheel0MotorID = 51;
        public static int flywheel1MotorID = 52;
        public static int flywheel2MotorID = 53;
        public static int flywheel3MotorID = 54;

        public static String telemetryNameRoot = "Shooter/";
        public static String telemetryNameFlywheelL = telemetryNameRoot+"Flywheel_L/";
        public static String telemetryNameFlywheelR = telemetryNameRoot+"Flywheel_R/";
        public static String telemetryNamesFlywheel[] = {telemetryNameFlywheelL, telemetryNameFlywheelR};
        public static String telemetryNameHood = telemetryNameRoot+"Hood/";

        public static String telemetryYAMSFlywheelL = "FlywheelL_";
        public static String telemetryYAMSFlywheelR = "FlywheelR_";
        public static String telemetryYAMSFlywheelList[] = {telemetryYAMSFlywheelL, telemetryYAMSFlywheelR};
    }

    public static class Indexer {
        public static int floorMotorID = 40;
        public static int ceilingMotorID = 41;
        public static int kickerMotorID = 42;

        public static String telemetryNameRoot = "Indexer/";
        public static String telemetryNameFloor = telemetryNameRoot+"Floor/";
        public static String telemetryNameCeiling = telemetryNameRoot+"Ceiling/";
        public static String telemetryNameKicker = telemetryNameRoot+"Kicker/";

        public static String telemetryYAMSName = "Roller_";
    }

    public static class Intake {
        public static int pivotMotorID = 30;
        public static int rollerMotorID = 31;

        public static String telemetryNameRoot = "Intake/";
        public static String telemetryNamePivot = telemetryNameRoot+"Pivot/";
        public static String telemetryNameRoller = telemetryNameRoot+"Roller/";

        public static String telemetryYAMSPivot = "Pivot_";
        public static String telemetryYAMSRoller = "Roller_";
    }

    public static class Vision {

    }
}
