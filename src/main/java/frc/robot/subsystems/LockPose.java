package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class LockPose implements SwerveRequest {
      public SwerveModuleState[] ModuleStates = {
         new SwerveModuleState(),
         new SwerveModuleState(),
         new SwerveModuleState(),
         new SwerveModuleState()
      };

      public LockPose() {
            super();

            for (int i = 0; i < ModuleStates.length; ++i) {
                  ModuleStates[i].angle = 
                     i % 2 == 0 ? Rotation2d.fromDegrees(45) : Rotation2d.fromDegrees(-45);
            }
      }

   @Override
   public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
      var moduleRequest = new SwerveModule.ModuleRequest()
         .withUpdatePeriod(parameters.updatePeriod);
      for (int i = 0; i < modulesToApply.length && i < ModuleStates.length; ++i) {
         modulesToApply[i].apply(moduleRequest.withState(ModuleStates[i]));
      }
      return StatusCode.OK;
   }
}
