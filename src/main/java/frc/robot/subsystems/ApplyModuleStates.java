package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ApplyModuleStates implements SwerveRequest {
   public SwerveModuleState[] ModuleStates = new SwerveModuleState[0];

   @Override
   public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
      var moduleRequest = new SwerveModule.ModuleRequest()
         .withUpdatePeriod(parameters.updatePeriod);
      for (int i = 0; i < modulesToApply.length && i < ModuleStates.length; ++i) {
         /* apply the SwerveModuleState to the module */
         modulesToApply[i].apply(moduleRequest.withState(ModuleStates[i]));
      }
      return StatusCode.OK;
   }
}
