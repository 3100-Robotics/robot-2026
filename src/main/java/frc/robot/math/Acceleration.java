package frc.robot.math;

import edu.wpi.first.wpilibj.Timer;

public class Acceleration {
    private double last = 0;
    private double current = 0;

    private double lastTime = 0;

    private double acceleration = 0;

    public Acceleration(double tolerance) {
        lastTime = Timer.getFPGATimestamp();
    }

    public void update(double newcurrent) {
        last = current;
        current = newcurrent;
        acceleration = (current - last) / (Timer.getFPGATimestamp() - lastTime);
        lastTime = Timer.getFPGATimestamp();
    }

    public double getAccel() {
        return acceleration;
    }
}
