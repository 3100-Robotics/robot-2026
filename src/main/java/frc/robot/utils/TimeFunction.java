package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TimeFunction {
    public static Runnable time(Runnable callback, String name) {
        Runnable func = () -> {
            double startTime = Timer.getFPGATimestamp();
            try {
                callback.run();
            } catch (Exception e) {
                e.printStackTrace();
            }
            double endTime = Timer.getFPGATimestamp();
            double executionTimeMs = (endTime - startTime) * 1000.0;
            SmartDashboard.putNumber(String.format("%s_time_exec", name), executionTimeMs);
        };
        return func;
    }
}
