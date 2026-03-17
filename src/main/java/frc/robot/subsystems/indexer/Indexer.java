package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Logging;
import frc.robot.subsystems.intake.RollerState;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public class Indexer extends SubsystemBase {
    SmartMotorControllerConfig baseRollerMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    // Floor
    private SmartMotorControllerConfig floorMotorConfig = baseRollerMotorConfig.clone()
        .withGearing(new MechanismGearing(GearBox.fromStages("20:24")))

        .withMotorInverted(true)

        .withClosedLoopController(0.01, 0, 0)
        .withSimClosedLoopController(0.05, 0, 0)

        .withFeedforward(new SimpleMotorFeedforward(0, 0.15, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0.1, 0))
    ;

    // Ceiling
    private SmartMotorControllerConfig ceilingMotorConfig = baseRollerMotorConfig.clone()
        .withClosedLoopController(0.01, 0, 0)
        .withSimClosedLoopController(0.01, 0, 0)

        .withFeedforward(new SimpleMotorFeedforward(0, 0.12, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0.105, 0))

        .withGearing(new MechanismGearing(GearBox.fromStages("20:24")));

    // Kicker
    private SmartMotorControllerConfig kickerMotorConfig = baseRollerMotorConfig.clone()
        .withClosedLoopController(0.01, 0, 0)
        .withSimClosedLoopController(0.01, 0, 0)

        .withFeedforward(new SimpleMotorFeedforward(0, 0.2, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0.105, 0))

        .withGearing(new MechanismGearing(GearBox.fromStages("36:24")));

    private TalonFX vendorFloorMotor = new TalonFX(Constants.Indexer.floorMotorID);
    private SparkMax vendorCeilingMotor = new SparkMax(Constants.Indexer.ceilingMotorID, MotorType.kBrushless);
    private SparkMax vendorKickerMotor = new SparkMax(Constants.Indexer.kickerMotorID, MotorType.kBrushless);

    private Roller floorRollers;
    private Roller ceilingRollers;
    private Roller kickerRollers;

    private Command dbg_stop;
    private Command dbg_runAll;

    private SpeedSet speeds = Constants.Indexer.off;
    private Supplier<SpeedSet> speedsProvider = () -> speeds;

    public Indexer() {
        floorRollers = new Roller(Constants.Indexer.Main.nameFloor, DCMotor.getKrakenX60(1), vendorFloorMotor, floorMotorConfig);
        ceilingRollers = new Roller(Constants.Indexer.Main.nameCeiling, DCMotor.getNEO(1), vendorCeilingMotor, ceilingMotorConfig);
        kickerRollers = new Roller(Constants.Indexer.Main.nameKicker, DCMotor.getNEO(1), vendorKickerMotor, kickerMotorConfig);

        dbg_stop = Commands.parallel(
            floorRollers.stop(),
            ceilingRollers.stop(),
            kickerRollers.stop()
        ).withName(Constants.join('/', Constants.Indexer.nameRoot, "stop"));

        dbg_runAll = Commands.parallel(
            floorRollers.runAtSpeed(RPM.of(2377)),
            ceilingRollers.runAtSpeed(RPM.of(4414)),
            kickerRollers.runAtSpeed(RPM.of(4414))
        ).withName(Constants.join('/', Constants.Indexer.nameRoot, "runAll"));

        Logging.registerDebugCommand(dbg_stop.getName(), dbg_stop);
        Logging.registerDebugCommand(dbg_runAll.getName(), dbg_runAll);

        
    }


    private Command setState() {
        return roller.run(() -> rollerStateProvider.get().speed);
    }

    private Command setStateSpecialOff() {
        return roller.set(0);
    }


    public Command on() {
        return Commands.runOnce(() -> rollerState = RollerState.on);
    }

    public Command off() {
        return Commands.runOnce(() -> rollerState = RollerState.off);
    }

    public Command toggle() {
        return Commands.runOnce(() -> {
            if (rollerState==RollerState.on) {
                rollerState = RollerState.off;
            } else {
                rollerState = RollerState.on;
            }
        });
    }


    public Command run() {
        return Commands.parallel(
            floorRollers.runAtSpeed(RPM.of(2377)),
            ceilingRollers.runAtSpeed(RPM.of(4414)),
            kickerRollers.runAtSpeed(RPM.of(4414))
        );
    }

    public Command runRev() {
        return Commands.parallel(
            floorRollers.runAtSpeed(RPM.of(-2377)),
            ceilingRollers.runAtSpeed(RPM.of(-4414)),
            kickerRollers.runAtSpeed(RPM.of(-4414))
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
            Constants.join('_', Constants.Indexer.Main.nameFloor, "RPM"),
            floorRollers.motor.getMechanismVelocity().in(RPM)
        );

        SmartDashboard.putNumber(
            Constants.join('_', Constants.Indexer.Main.nameCeiling, "RPM"),
            ceilingRollers.motor.getMechanismVelocity().in(RPM)
        );

        SmartDashboard.putNumber(
            Constants.join('_', Constants.Indexer.Main.nameKicker, "RPM"),
            kickerRollers.motor.getMechanismVelocity().in(RPM)
        );

        // SmartDashboard.putNumber("kickerAmps", kickerRollers.motor.getSupplyCurrent().get().in(Amps));
    }
}
