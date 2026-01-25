package frc.robot.subsystems;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private TalonFXConfiguration intakeMotorConfiguration = new TalonFXConfiguration()
        .withAudio(new AudioConfigs()
            .withBeepOnBoot(true)
            .withBeepOnConfig(true))
        ;

    private TalonFX intakeMotor = new TalonFX(32);

    public Command intake = intake().withName("dbgcommands/runintake");


    public Intake() {
        intakeMotor.getConfigurator().apply(intakeMotorConfiguration);
    }

    private Command intake() {
        return run(() -> intakeMotor.set(1));
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(intake);
    }
}
