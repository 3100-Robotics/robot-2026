package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Locator;
import frc.robot.utils.Acceleration;

public class Shooter extends SubsystemBase {
    /*
     * Okay, so Okay :}
     * 
     * We split the functions of the shooter into
     * two flywheels and a hood because of how YAMS
     * functions
     */

    private Angle hoodAngle = Constants.Shooter.minHoodAngle;
    private AngularVelocity flywheelSpeed = RPM.of(0);

    public Supplier<Angle> angleProvider = () -> hoodAngle;
    public Supplier<AngularVelocity> speedProvider = () -> flywheelSpeed;

    public final Hood hood = new Hood();
    public final DoubleFlywheel flywheelL = new DoubleFlywheel(0, 51, 52, true, 0.003, 0.137);
    public final DoubleFlywheel flywheelR = new DoubleFlywheel(1, 53, 54, false, 0.003, 0.1399);

    public final Acceleration flywheelLAccel = new Acceleration(1);
    public final Acceleration flywheelRAccel = new Acceleration(1);

    public final Trigger flywheelsAtRPM = 
        new Trigger(
            () -> flywheelL.flywheel.getSpeed()
                .isNear(speedProvider.get(), RPM.of(100)) &&
                flywheelR.flywheel.getSpeed()
                .isNear(speedProvider.get(), RPM.of(100))
            )
    ;

    public final Trigger flywheelsAtRPMAcceleration = 
        new Trigger(
            () -> flywheelRAccel.getAccel() < 1000 && flywheelLAccel.getAccel() < 1000
                
            ).debounce(0.05)
    ;

    public boolean keepSpunUpBoolean = false;
    public final Trigger keepSpunUpTrigger = new Trigger(() -> keepSpunUpBoolean);

    public Shooter() {
        flywheelR.setDefaultCommand(flywheelR.flywheel.run(speedProvider));
        flywheelL.setDefaultCommand(flywheelL.flywheel.run(speedProvider));
        hood.setDefaultCommand(hood.hood.setAngle(Constants.Shooter.minHoodAngle));

        SmartDashboard.putNumber("testingRPM", 1000);
        SmartDashboard.putNumber("testingANGLE", 13);
    }

    public Command toggleKeepSpunUp() {
        return Commands.runOnce(() -> {
                if (keepSpunUpBoolean==true) {
                    keepSpunUpBoolean = false;
                } else {
                    keepSpunUpBoolean = true;
                }
            }
        );
    }

    public Pair<Angle, AngularVelocity> calculateFireAngleAndSpeed() {
        var shooterDistToHub = 
            Locator.getInstance().distanceToHub.get() // Robot center distance from hub
            .plus(Inches.of(5.4330709)) // Center to fuel exit
        ;
        var angleTable = Constants.Shooter.Physical.distanceAngleTable;
        var speedTable = Constants.Shooter.Physical.distanceSpeedTable;

        double angleOut = 15; // Default
        if (shooterDistToHub.lt(angleTable.get(0).getFirst())) {
            angleOut = Constants.Shooter.minHoodAngle.in(Degrees);
        }

        if (shooterDistToHub.gt(angleTable.get(angleTable.size()-1).getFirst())) {
            angleOut = Constants.Shooter.maxHoodAngle.in(Degrees);
        }

        for (int i = 0; i < Constants.Shooter.Physical.distanceAngleTable.size()-1; i++) {
            if (shooterDistToHub.gte(angleTable.get(i).getFirst()) && 
                shooterDistToHub.lte(angleTable.get(i+1).getFirst())
            ) {
                var x1 = angleTable.get(i).getFirst().in(Feet);
                var x2 = angleTable.get(i+1).getFirst().in(Feet);
                var y1 = angleTable.get(i).getSecond();
                var y2 = angleTable.get(i+1).getSecond();

                var m = (y2 - y1) / (x2 - x1);
                var b = y1 - m * x1;
                angleOut = m * shooterDistToHub.in(Feet) + b;
            }
        }

        
        double speedOut = 4000;
        if (shooterDistToHub.lt(speedTable.get(0).getFirst())) {
            speedOut = 31;
        }

        if (shooterDistToHub.gt(speedTable.get(angleTable.size()-1).getFirst())) {
            speedOut = 3100;
        }

        for (int i = 0; i < Constants.Shooter.Physical.distanceAngleTable.size()-1; i++) {
            if (shooterDistToHub.gte(speedTable.get(i).getFirst()) && 
                shooterDistToHub.lte(speedTable.get(i+1).getFirst())
            ) {
                var x1 = speedTable.get(i).getFirst().in(Feet);
                var x2 = speedTable.get(i+1).getFirst().in(Feet);
                var y1 = speedTable.get(i).getSecond();
                var y2 = speedTable.get(i+1).getSecond();

                var m = (y2 - y1) / (x2 - x1);
                var b = y1 - m * x1;
                speedOut = m * shooterDistToHub.in(Feet) + b;
            }
        }

        return Pair.of(Degrees.of(angleOut), RPM.of(speedOut));
    }

    // get/set hood setpoint
    public void setHoodAngleSetpoint(Angle newAngle) {
        if (newAngle.in(Degrees) > Constants.Shooter.maxHoodAngle.in(Degrees)) {
            hoodAngle = Constants.Shooter.maxHoodAngle;
        } else if (newAngle.in(Degrees) < Constants.Shooter.minHoodAngle.in(Degrees)) {
            hoodAngle = Constants.Shooter.minHoodAngle;
        } else {
            hoodAngle = newAngle;
        }
    }

    public Angle getHoodAngleSetpoint() {
        return hoodAngle;
    }

    // get/set flywheel setpoint
    public void setSpeedSetpoint(AngularVelocity newSpeed) {
        flywheelSpeed = newSpeed;
    }

    public AngularVelocity getSpeedSetpoint() {
        return flywheelSpeed;
    }


    @Override
    public Command idle() {
        return Commands.parallel(
            stopHood(),
            idleFlywheels()
        );
    }

    public Command stopHood() {
        return hood.hood.runTo(hood.hood.getAngle(), Degrees.of(360))
            .andThen(hood.hood.set(0));
    }

    public Command goToCurrentAngle() {
        return hood.hood.setAngle(angleProvider);
    }


    public Command stopFlywheels() {
        return flywheelL.flywheel.set(0)
            .alongWith(flywheelR.flywheel.set(0));
    }

    public Command idleFlywheels() {
        return Commands.runOnce(() -> {
            flywheelSpeed = RPM.of(0);
        });
    }


    public Command runFlywheelsToCurrent() {
        return flywheelL.flywheel.setSpeed(speedProvider)
            .alongWith(flywheelR.flywheel.setSpeed(speedProvider));
    }


    @Override
    public void periodic() {
        flywheelLAccel.update(flywheelL.flywheel.getMotor().getMechanismVelocity().in(RPM));
        flywheelRAccel.update(flywheelR.flywheel.getMotor().getMechanismVelocity().in(RPM));

        SmartDashboard.putNumber("flywheelLAccel", flywheelLAccel.getAccel());
        SmartDashboard.putNumber("flywheelRAccel", flywheelRAccel.getAccel());
        SmartDashboard.putNumber("flywheelsAtAccel", flywheelsAtRPMAcceleration.getAsBoolean() ? 1000 : 0);

        SmartDashboard.putNumber("flywheelSpeed", flywheelSpeed.in(RPM));

        SmartDashboard.putNumber("hoodAngle2", hood.hood.getAngle().in(Degrees));
        SmartDashboard.putNumber("flywheel left rpm", flywheelL.flywheel.getMotor().getMechanismVelocity().in(RPM));
        SmartDashboard.putNumber("flywheel right rpm", flywheelR.flywheel.getMotor().getMechanismVelocity().in(RPM));

        SmartDashboard.putBoolean("flywheelatrpm", flywheelsAtRPM.getAsBoolean());
        SmartDashboard.putNumber("fromHoodSupllierAngle", angleProvider.get().in(Degrees));
        SmartDashboard.putNumber("fromHoodSupllierRPM", speedProvider.get().in(RPM));
    }
}
