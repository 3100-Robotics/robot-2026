package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    // floorMotor is the main motor that the others follow
    private SparkMax floorMotor = new SparkMax(40, MotorType.kBrushless);
    private SparkMax ceilingMotor  = new SparkMax(41, MotorType.kBrushless);
    private SparkMax verticalMotor = new SparkMax(42, MotorType.kBrushless);

    private SparkBaseConfig rootConfig = new SparkMaxConfig()
        .voltageCompensation(12)
        .inverted(false)
        .smartCurrentLimit(30)
        .apply(
            new ClosedLoopConfig()
            .p(1)
            .i(0)
            .d(0)
            .apply(new FeedForwardConfig()
                .kS(0)
                .kG(0)
                .kV(0)
            )
        )
    ;

    private SparkBaseConfig ceilingMotorConfig = new SparkMaxConfig()
        .apply(rootConfig)
        .inverted(true)
    ;

    private SparkBaseConfig verticalMotorConfig = new SparkMaxConfig()
        .apply(rootConfig)
        .inverted(true)
    ;

    public Indexer() {

    }
}
