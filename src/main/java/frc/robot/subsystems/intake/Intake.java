package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Logging;

/*
 * Bird == Flipper == Translation
 */
public class Intake extends SubsystemBase {
    private final Pivot pivotSub = new Pivot();
    private final Roller rollerSub = new Roller();

    public boolean deployed = true;
    public Trigger isDeployed = new Trigger(() -> deployed);

    public DoubleLogEntry rollerCurrent;

    public Intake() {
        setName("Intake");

        rollerCurrent = new DoubleLogEntry(Logging.getLTInstance().m_log0, "/intake/rollercurrent");
        isDeployed.whileTrue(deploy()).whileFalse(stow());
    }

    // public Command deployState() {
        
    // }

    public Command stow() {
        return pivotSub.pivot.setAngle(Constants.Intake.pivotStowAngle);
    }
    
    public Command deploy() {
        return pivotSub.pivot.setAngle(Constants.Intake.pivotDeployAngle);
    }

    public Command stop() {
        return rollerSub.roller.set(0);
    }

    public Command runAtSpeed(AngularVelocity speed) {
        return rollerSub.roller.setSpeed(speed);
    }

    @Override
    public void periodic() {
        rollerSub.roller.getMotorController().getMechanismSetpointVelocity()
            .ifPresent(
                setpoint -> SmartDashboard.putNumber(Constants.Intake.telemetryNameRoller+"RPM_Setpoint", setpoint.in(RPM))
            );
        
        SmartDashboard.putNumber(
            Constants.Intake.telemetryNameRoller+"RPM_Speed",
            rollerSub.roller.getSpeed().in(RPM)
        );
    
        SmartDashboard.putNumber("angleintakepivot", pivotSub.pivot.getAngle().in(Degrees));

        pivotSub.pivot.getMechanismSetpoint()
            .ifPresent(
                setpoint -> SmartDashboard.putNumber(
                    Constants.Intake.telemetryNamePivot+"setpoint", setpoint.in(Degrees)
                )
            );

        if (Logging.getLTInstance().doLogging) {
            rollerSub.roller.getMotor().getSupplyCurrent().ifPresent(current -> rollerCurrent.append(current.in(Amps)));
        }
    }
}
