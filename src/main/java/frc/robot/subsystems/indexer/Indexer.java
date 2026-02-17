package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

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
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public class Indexer extends SubsystemBase {
    SmartMotorControllerConfig baseRollerMotorConfig = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)

        // .withClosedLoopController(4, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        // .withSimClosedLoopController(0.01, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))

        // .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        // .withSimFeedforward(new SimpleMotorFeedforward(0, 0.2, 0))

        // .withMomentOfInertia(KilogramSquareMeters.of(0.000001))

        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopRampRate(Seconds.of(0.25))
        .withOpenLoopRampRate(Seconds.of(0.25));

    /*
     * The diameter and mass are used for calculating the MOI of the system. 
     * Until we can get real physical values, these are complete trash and
     * any pid tuned from them will be nowhere near where we need it  
     */
    // FlyWheelConfig baseRollerConfig = new FlyWheelConfig()
    //     .withDiameter(Inches.of(4))
    //     .withMass(Pounds.of(1))
    //     .withUpperSoftLimit(RPM.of(6000))
    // ;

    private SmartMotorControllerConfig floorMotorConfig = baseRollerMotorConfig.clone()
        .withGearing(new MechanismGearing(GearBox.fromStages("20:24")))

        .withMotorInverted(true)
    
        .withClosedLoopController(0.01, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(0.05, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))

        .withFeedforward(new SimpleMotorFeedforward(0, 0.15, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0.1, 0))
    ;

    private SmartMotorControllerConfig ceilingMotorConfig = baseRollerMotorConfig.clone()
        .withClosedLoopController(0.01, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(0.01, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))

        .withFeedforward(new SimpleMotorFeedforward(0, 0.12, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0.105, 0))

        .withGearing(new MechanismGearing(GearBox.fromStages("20:24")));

    private SmartMotorControllerConfig kickerMotorConfig = baseRollerMotorConfig.clone()
        .withClosedLoopController(0.01, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
        .withSimClosedLoopController(0.01, 0, 0)//, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))

        .withFeedforward(new SimpleMotorFeedforward(0, 0.2, 0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0.105, 0))

        .withGearing(new MechanismGearing(GearBox.fromStages("36:24")));

    private TalonFX vendorFloorMotor = new TalonFX(Constants.Indexer.floorMotorID);
    private SparkMax vendorCeilingMotor = new SparkMax(Constants.Indexer.ceilingMotorID, MotorType.kBrushless);
    private SparkMax vendorKickerMotor = new SparkMax(Constants.Indexer.kickerMotorID, MotorType.kBrushless);

    private Roller floorRollers;
    private Roller ceilingRollers;
    private Roller kickerRollers;

    public Command stop;
    private Command runAll;

    public Indexer() {
        floorRollers = new Roller(Constants.Indexer.Main.nameFloor, DCMotor.getKrakenX60(1), vendorFloorMotor, floorMotorConfig);
        ceilingRollers = new Roller(Constants.Indexer.Main.nameCeiling, DCMotor.getNEO(1), vendorCeilingMotor, ceilingMotorConfig);
        kickerRollers = new Roller(Constants.Indexer.Main.nameKicker, DCMotor.getNEO(1), vendorKickerMotor, kickerMotorConfig);

        stop = Commands.parallel(
            floorRollers.stop(),
            ceilingRollers.stop(),
            kickerRollers.stop()
        ).withName(Constants.join('/', Constants.Indexer.nameRoot, "stop"));

        runAll = Commands.parallel(
            floorRollers.runAtSpeed(RPM.of(2377)),
            ceilingRollers.runAtSpeed(RPM.of(4414)),
            kickerRollers.runAtSpeed(RPM.of(4414))
        ).withName(Constants.join('/', Constants.Indexer.nameRoot, "runAll"));

        Logging.registerDebugCommand(stop.getName(), stop);
        Logging.registerDebugCommand(runAll.getName(), runAll);
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
    }
}
