package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
    

    public Rollers() {
        
    }

    private Command intake() {
        return run(() -> {});
        // return run(() -> intakeMotor.set(1));
    }

    @Override
    public void periodic() {
        // SmartDashboard.putData(intake);
    }
}
