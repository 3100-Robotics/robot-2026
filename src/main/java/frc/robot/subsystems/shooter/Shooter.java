package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Locator;

public class Shooter extends SubsystemBase {
    /*
     * Okay, so Okay :}
     *
     * We split the functions of the shooter into
     * two flywheels and a hood because of how YAMS
     * functions
     */

    public static Pair<Angle, AngularVelocity> calculateFireAngleAndSpeed() {
        var shooterDistToHub =
                Locator.getInstance()
                        .distanceToHub
                        .get() // Robot center distance from hub
                        .plus(Inches.of(5.4330709)) // Center to fuel exit
                ;
        var angleTable = Constants.Shooter.Physical.distanceAngleTable;
        var speedTable = Constants.Shooter.Physical.distanceSpeedTable;

        double angleOut = 15; // Default
        if (shooterDistToHub.lt(angleTable.get(0).getFirst())) {
            angleOut = Constants.Shooter.minHoodAngle.in(Degrees);
        }

        if (shooterDistToHub.gt(angleTable.get(angleTable.size() - 1).getFirst())) {
            angleOut = Constants.Shooter.maxHoodAngle.in(Degrees);
        }

        for (int i = 0; i < Constants.Shooter.Physical.distanceAngleTable.size() - 1; i++) {
            if (shooterDistToHub.gte(angleTable.get(i).getFirst())
                    && shooterDistToHub.lte(angleTable.get(i + 1).getFirst())) {
                var x1 = angleTable.get(i).getFirst().in(Feet);
                var x2 = angleTable.get(i + 1).getFirst().in(Feet);
                var y1 = angleTable.get(i).getSecond();
                var y2 = angleTable.get(i + 1).getSecond();

                var m = (y2 - y1) / (x2 - x1);
                var b = y1 - m * x1;
                angleOut = m * shooterDistToHub.in(Feet) + b;
            }
        }

        double speedOut = 4000;
        if (shooterDistToHub.lt(speedTable.get(0).getFirst())) {
            speedOut = 31;
        }

        if (shooterDistToHub.gt(speedTable.get(angleTable.size() - 1).getFirst())) {
            speedOut = 3100;
        }

        for (int i = 0; i < Constants.Shooter.Physical.distanceAngleTable.size() - 1; i++) {
            if (shooterDistToHub.gte(speedTable.get(i).getFirst())
                    && shooterDistToHub.lte(speedTable.get(i + 1).getFirst())) {
                var x1 = speedTable.get(i).getFirst().in(Feet);
                var x2 = speedTable.get(i + 1).getFirst().in(Feet);
                var y1 = speedTable.get(i).getSecond();
                var y2 = speedTable.get(i + 1).getSecond();

                var m = (y2 - y1) / (x2 - x1);
                var b = y1 - m * x1;
                speedOut = m * shooterDistToHub.in(Feet) + b;
            }
        }

        return Pair.of(Degrees.of(angleOut), RPM.of(speedOut));
    }

    // get/set hood setpoint

    // public void setHoodAngleSetpoint(Angle newAngle) {
    //     if (newAngle.in(Degrees) > Constants.Shooter.maxHoodAngle.in(Degrees)) {
    //         hoodAngle = Constants.Shooter.maxHoodAngle;
    //     } else if (newAngle.in(Degrees) < Constants.Shooter.minHoodAngle.in(Degrees)) {
    //         hoodAngle = Constants.Shooter.minHoodAngle;
    //     } else {
    //         hoodAngle = newAngle;
    //     }
    // }

    // public Angle getHoodAngleSetpoint() {
    //     return hoodAngle;
    // }

    // get/set flywheel setpoint
    // public void setSpeedSetpoint(AngularVelocity newSpeed) {
    //     flywheelSpeed = newSpeed;
    // }

    // public AngularVelocity getSpeedSetpoint() {
    //     return flywheelSpeed;
    // }

    // public Command stopHood() {
    //     return hood.hood.runTo(hood.hood.getAngle(), Degrees.of(360))
    //         .andThen(hood.hood.set(0));
    // }

    // public Command goToCurrentAngle() {
    //     return hood.hood.setAngle(angleProvider);
    // }

    // public Command stopFlywheels() {
    //     return flywheelL.flywheel.set(0)
    //         .alongWith(flywheelR.flywheel.set(0));
    // }

    // public Command idleFlywheels() {
    //     return Commands.runOnce(() -> {
    //         flywheelSpeed = RPM.of(1000);
    //     });
    // }

    // public Command runFlywheelsToCurrent() {
    //     return flywheelL.flywheel.setSpeed(speedProvider)
    //         .alongWith(flywheelR.flywheel.setSpeed(speedProvider));
    // }
}
