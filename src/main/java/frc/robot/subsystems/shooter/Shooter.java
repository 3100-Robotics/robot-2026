package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    /*
     * Okay, so Okay :}
     * 
     * We split the functions of the shooter into
     * two flywheels and a hood because of how YAMS
     * functions
     */
    private final Hood hood = new Hood();
    private final Flywheel flywheelL = new Flywheel(51, 52);
    private final Flywheel flywheelR = new Flywheel(53, 54);
}
