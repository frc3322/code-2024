// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
<<<<<<< HEAD
=======
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
>>>>>>> 41cafd2af535c21ffc5061cc87039cb1e3ce422c
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DriveConstants;
<<<<<<< HEAD
=======
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FieldConstants;
>>>>>>> 41cafd2af535c21ffc5061cc87039cb1e3ce422c
import frc.utils.SwerveUtils;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase implements Loggable{
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      CANIds.kFrontLeftDrivingCanId,
      CANIds.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      CANIds.kFrontRightDrivingCanId,
      CANIds.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      CANIds.kRearLeftDrivingCanId,
      CANIds.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      CANIds.kRearRightDrivingCanId,
      CANIds.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();
  // private final ADIS16470_IMU gyro2 = new ADIS16470_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  
  private double lastDir = 0;
  

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
<<<<<<< HEAD
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
   
=======
    DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(getAngle()),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    }
  );

  @Log
  public Field2d field = new Field2d();

  PIDController thetaController = new PIDController(
    AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController
  );
  

  // Path strings
  private String ampLineupPathName = "AmpLineup";

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    estimatedPose = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics, 
      Rotation2d.fromDegrees(getAngle()), 
      getModulePositions(), getPose()
      );
    

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetEstimatedPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      AutoConstants.holonomicPathFollowerConfig,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );

    thetaController.enableContinuousInput(-180, 180);

    SmartDashboard.putData(thetaController);

>>>>>>> 41cafd2af535c21ffc5061cc87039cb1e3ce422c
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
       


    SmartDashboard.updateValues();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Returns the corrected yaw for the robot
  public double getAngle() {
    double yaw = DriveConstants.kGyroReversed ? -m_gyro.getAngle() : m_gyro.getAngle();
    return yaw;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));


      if(inputTranslationMag == 0){
        inputTranslationDir = lastDir;
      }
      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      //ISTG MAKE SURE TO CHANGE THIS BACK TO ACTUAL RATE FROM CONSTANTS
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
 
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI); 
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

      lastDir = inputTranslationDir;
    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);


    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

<<<<<<< HEAD
=======
  /** Drives the robot relative to the robot when given chassis speeds
   * 
   * @param speeds The robot relative desired chassis speeds
  */
  public void autoDrive(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);


    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void drive(double xSpeed, double ySpeed, DoubleSupplier rot, boolean fieldRelative, boolean rateLimit) {
    this.drive(xSpeed, ySpeed, rot.getAsDouble(), fieldRelative, rateLimit);
  }

  /**
   * Follows a PathPlanner auton path.
   * 
   * @param path The path the robot should follow
   * @return The command to follow the path
   */
  public Command followAutonPath(PathPlannerPath path){
    return new FollowPathHolonomic(
      path, //Path from params
      this::getPose, // Robot pose supplier
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      AutoConstants.holonomicPathFollowerConfig,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Pathfinds to the start of the given path, then follows that path.
   * 
   * @param path The path to move to and follow
   * @return The pathfinding and following command
   */
  public Command pathfindThenFollowPath(PathPlannerPath path) {
    return new PathfindThenFollowPathHolonomic(
        path,
        AutoConstants.constraints,
        this::getPose,
        this::getRobotRelativeSpeeds,
        this::autoDrive,
        AutoConstants.holonomicPathFollowerConfig, // HolonomicPathFollwerConfig, see the API or "Follow a single path" example for more info
        0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            return isAllianceRed();
        },
        this // Reference to drive subsystem to set requirements
      );
  }

  /**
   * Returns command to drive to amp.
   * 
   * @return Dynamic trajectory to drive to amp
   */
  public Command AmpLineupDynamicTrajectory() {
    return pathfindThenFollowPath(
      PathPlannerPath.fromPathFile(ampLineupPathName)
    );
  }



  /*◇─◇──◇─◇
     Setters
  ◇─◇──◇─◇*/
  
  public void setVisionSystem(LimeLightVision vision){
    this.vision = vision;
  }
>>>>>>> 41cafd2af535c21ffc5061cc87039cb1e3ce422c
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
<<<<<<< HEAD
}
=======

  /** 
    * Returns the current angle of the robot 
    *
    * @return The current yaw of the robot, in degrees
  */
  
  public Pose2d getShooterPose(){
    Pose2d pose = isAllianceRed() ? FieldConstants.redCenterShootPose : FieldConstants.blueCenterShootPose;
    return pose;
  }

  public boolean atShootPose(){
    return atPose(getShooterPose(), ShooterConstants.shooterRevUpDistance, 0);
  }

  @Log
  public double getAngle() {
    double yaw = DriveConstants.kGyroReversed ? -m_gyro.getAngle() : m_gyro.getAngle();
    return yaw;
  }

  @Log
  public double getEstimatedAngle() {
    double rotation = estimatedPose.getEstimatedPosition().getRotation().getDegrees();
    double yaw = DriveConstants.kGyroReversed ? rotation : -rotation;
    return yaw;
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition fl = m_frontLeft.getPosition();
    SwerveModulePosition fr = m_frontRight.getPosition();
    SwerveModulePosition rl = m_rearLeft.getPosition();
    SwerveModulePosition rr = m_rearRight.getPosition();
    return new SwerveModulePosition[] {fl, fr, rl, rr};
  }

  public boolean atPose(Pose2d pose, double translationThreshold, double rotationThreshold){
    boolean translationInThreshold = atTranslation(pose.getTranslation(), translationThreshold);
    boolean rotationInThreshold = atRotation(pose.getRotation(), rotationThreshold);

    if (rotationThreshold == 0) {
      return translationInThreshold;
    }
    if (translationThreshold == 0) {
      return rotationInThreshold;
    }

    return translationInThreshold && rotationInThreshold;
  }


  public boolean atTranslation(Translation2d translation, double threshold){
    Translation2d robotTranslation = getPose().getTranslation();

    return robotTranslation.getDistance(translation) < threshold;
  }
  

  public boolean atRotation(Rotation2d rotation, double threshold){
    Rotation2d robotRotation = getPose().getRotation();
    
    double rotOffset = robotRotation.getDegrees() - rotation.getDegrees();

    return Math.abs(rotOffset) < threshold;
  }

  public boolean isAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  @Log
  public double getAngleToShooter() {
    Translation2d speakerPose = isAllianceRed() ? FieldConstants.redSpeakerTranslation : FieldConstants.blueSpeakerTranslation;
    Pose2d robotPose = estimatedPose.getEstimatedPosition();

    double calculatedAngle = Math.atan2(robotPose.getX()-speakerPose.getX(), robotPose.getY()-speakerPose.getY());

    return 270 - Units.radiansToDegrees(calculatedAngle);
  }

  public double getOutputToAngle(){
    double output = thetaController.calculate(getEstimatedAngle());
    if (output > 1) {
      output = 1;
    }
    if (output < -1) {
      output = -1;
    }

    return output;
  }
  
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    SmartDashboard.putString("robot pose, just odometry", LimeLightVision.poseAsString(getPose()));
    SmartDashboard.putString("Limelight in drivetrain", LimeLightVision.poseAsString(vision.getLimeLightAverage()));
    SmartDashboard.putString("wpilib estimated pose w/ ll", LimeLightVision.poseAsString(estimatedPose.getEstimatedPosition()));
    SmartDashboard.updateValues();
  
    // updates pose with current time, rotation, and module positions.
    estimatedPose.updateWithTime(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(getAngle()), getModulePositions());

    // updates pose with Lime Light positions
    if (vision.hasLeftTarget())
    {
      estimatedPose.addVisionMeasurement(vision.getLeftPose(), Timer.getFPGATimestamp());
    }

    if (vision.hasRightTarget())
    {
      estimatedPose.addVisionMeasurement(vision.getRightPose(), Timer.getFPGATimestamp());
    }
    
    field.setRobotPose(estimatedPose.getEstimatedPosition());
    this.resetOdometry(estimatedPose.getEstimatedPosition());

    thetaController.setSetpoint(getAngleToShooter());
  }
}
>>>>>>> 41cafd2af535c21ffc5061cc87039cb1e3ce422c
