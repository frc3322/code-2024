package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class Trajectories {
    
    /*◇─◇──◇─◇
       Configs
    ◇─◇──◇─◇*/
    
    static TrajectoryConfig defaultConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    static TrajectoryConfig slowConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquaredSlow)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    /*◇─◇──◇─◇
     Controllers
    ◇─◇──◇─◇*/
    
    public static ProfiledPIDController getDefaultThetaController(){
        
        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        return thetaController;
    }

    /*◇─◇──◇─◇
    Trajectories
    ◇─◇──◇─◇*/

    public static Trajectory slowForward4Meters() {

        return TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(2, 0)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(4, 0, new Rotation2d(0)),
            slowConfig);

    }

}

