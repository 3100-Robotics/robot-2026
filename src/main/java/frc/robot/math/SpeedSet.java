package frc.robot.math;

import edu.wpi.first.units.measure.AngularVelocity;

public class SpeedSet {
    public final AngularVelocity kickerSpeed;
    public final AngularVelocity ceilingSpeed;
    public final AngularVelocity floorSpeed;

    public SpeedSet(
            AngularVelocity kickerSpeed, AngularVelocity ceilingSpeed, AngularVelocity floorSpeed) {
        this.kickerSpeed = kickerSpeed;
        this.ceilingSpeed = ceilingSpeed;
        this.floorSpeed = floorSpeed;
    }
}
