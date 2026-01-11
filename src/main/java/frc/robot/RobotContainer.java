// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Break out auton stuff to a seperate file

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstantsV0;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.util.datalog.StringLogEntry;

public class RobotContainer {

  private final DataLog m_log0 = DataLogManager.getLog();
  private final DataLog m_log1 = DataLogManager.getLog();
  private final StringLogEntry testStringLog = new StringLogEntry(m_log0, "/root/teststring");

  private final CommandXboxController m_driverCtl = new CommandXboxController(0);
  private final CommandXboxController m_coDriverCtl = new CommandXboxController(1);

  private AutoChooser m_autoChooser = new AutoChooser();
  
  // Drivetrain
  private final double m_maxSpeed = TunerConstantsV0.kSpeedAt12Volts.in(MetersPerSecond); // Get real max speed
  private final double m_maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final SwerveRequest.FieldCentric m_driveFieldCentric = new SwerveRequest.FieldCentric()
    .withDeadband(m_maxSpeed * 0.09)
    .withRotationalDeadband(m_maxAngularRate * 0.09) // Experiment with best values for these but they worked on Hawksbill
    .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage); 
  private Drivetrain m_drivetrain = TunerConstantsV0.createDrivetrain();

  public RobotContainer() {
    DriverStation.startDataLog(m_log1);

    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    configureDashboard();
    configureBindings();
    configureActions();
  }

  private void configureDashboard() {
    SmartDashboard.putData("autoChooser", m_autoChooser);

    m_autoChooser.addCmd("mustard auton", () -> Commands.run(()->testStringLog.append("Mustard!")));
    // m_autoChooser.addRoutine("Mustard");
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
      m_drivetrain.applyRequest(
        () -> {return m_driveFieldCentric
          .withVelocityX(m_driverCtl.getLeftX())
          .withVelocityY(m_driverCtl.getLeftY())
          .withRotationalRate(m_driverCtl.getRightX());
        }
    ));
  }

  private void configureActions() {
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
