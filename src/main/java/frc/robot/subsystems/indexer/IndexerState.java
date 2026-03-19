package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

public enum IndexerState {
    onFloor(Constants.Indexer.floorSpeed),
    onCeiling(Constants.Indexer.ceilingSpeed),
    onKicker(Constants.Indexer.kickerSpeed),
    off(RPM.of(0))
    ;

    public final AngularVelocity speed;

    IndexerState(AngularVelocity speed) {
        this.speed = speed;
    }
}
