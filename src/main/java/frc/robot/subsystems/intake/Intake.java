package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utils.PivotState;
import frc.robot.Logging;

/*
 * Bird == Flipper == Translation
 */
public class Intake extends SubsystemBase {
    public final Pivot pivotSub;
    public final Roller rollerSub = new Roller();

    public PivotState pivotState = PivotState.Stow; 
    public Supplier<PivotState> pivotStateSupplier = () -> pivotState;

    // public boolean deployed = false;
    // public Trigger isDeployed = new Trigger(() -> deployed);

    public DoubleLogEntry rollerCurrent;

    public Intake() {
        setName("Intake");

        rollerCurrent = new DoubleLogEntry(Logging.getLTInstance().m_log0, "/intake/rollercurrent");
        pivotSub = new Pivot(pivotStateSupplier);
        // isDeployed.whileTrue(deploy()).whileFalse(stow());
    }

    public Command toggleDeploy() {
        return Commands.runOnce(() -> {
            if (pivotState==PivotState.Stow) {
                pivotState = PivotState.Deployed;
            } else if (pivotState==PivotState.Deployed) {
                pivotState = PivotState.Stow;
            } else if (pivotState==PivotState.SuperStow) {
                pivotState = PivotState.Stow;
            }
        });
    }

    public Command setSuperStow() {
        return Commands.runOnce(() -> {
            pivotState = PivotState.SuperStow;
        });
    }

    // public Command setDeploy() {
    //     return Commands.runOnce(() -> deployed = true);
    // }

    // public Command setStow() {
    //     return Commands.runOnce(() -> deployed = false);
    // }

    public Command stow() {
        return pivotSub.pivot.setAngle(Constants.Intake.pivotStowAngle);
    }

    public Command halfway() {
        return pivotSub.pivot.setAngle(Degrees.of(50));
    }

    public Command superStow() {
        return pivotSub.pivot.setAngle(Constants.Intake.pivotSuperStowAngle);
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
        // SmartDashboard.putBoolean("thinksDeployed", isDeployed.getAsBoolean());
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
