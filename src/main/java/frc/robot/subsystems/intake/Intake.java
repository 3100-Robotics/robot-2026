package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final Pivot pivot = new Pivot();
    private final Rollers rollers = new Rollers();
}
