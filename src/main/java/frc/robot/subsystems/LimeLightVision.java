// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimeLightConstants;

public class LimeLightVision extends SubsystemBase {
  /** Creates a new LimeLightVision. */
  public Pose2d limelightPose;
  public LimeLightVision() {
  }

  private static NetworkTableEntry getTableEntry(String limeLight, String entryKey) {
    return NetworkTableInstance.getDefault().getTable(limeLight).getEntry(entryKey);
  }
  /**
   * Gets the pose from the left Limelight
   *
   * @return The pose retrieved from LimeLight.
   */
  public Pose2d getLeftPose(){
    double[] limeLightData = getTableEntry("limelight-left", "botpose").getDoubleArray(new double[6]);
    return new Pose2d(limeLightData[0] + LimeLightConstants.fieldLengthOffset, limeLightData[1] + LimeLightConstants.fieldWidthOffset, Rotation2d.fromDegrees(limeLightData[5]));
  }
  /**
   * Gets the pose from the right Limelight
   *
   * @return The pose retrieved from LimeLight.
   */
  public Pose2d getRightPose(){
    double[] limeLightData = getTableEntry("limelight-right", "botpose").getDoubleArray(new double[6]);
    return new Pose2d(limeLightData[0] + LimeLightConstants.fieldLengthOffset, limeLightData[1] + LimeLightConstants.fieldWidthOffset, Rotation2d.fromDegrees(limeLightData[5]));
  }
  /**
   * Gets the average pose from the Limelight vision system.
   *
   * @return The average pose retrieved from both LimeLights.
   */

  public Pose2d getLimeLightAverage(){
    if (hasLeftTarget() && hasRightTarget()){
      Pose2d left = getLeftPose();
      Pose2d right = getRightPose();
      Pose2d[] poses = {left, right};
      return averagePoses(poses);
    } else if (hasRightTarget()){
      Pose2d right = getRightPose();
      Pose2d[] poseArray = {right};
      return averagePoses(poseArray);
    }
    else if (hasLeftTarget()){
      Pose2d left = getLeftPose();
      Pose2d[] poseArray = {left};
      return averagePoses(poseArray);
    }
    return new Pose2d();
  }
  /**
   * Gets the target ID from the left LimeLight.
   *
   * @return The target ID.
   */
  public double getLeftTagID(){
    if (hasLeftTarget()){
        double[] tag = getTableEntry("limelight-left", "tid").getDoubleArray(new double[6]);
        return tag[0];
    }
    return -1;
  }
    /**
   * Gets the target ID from the right LimeLight.
   *
   * @return The target ID.
   */
  public double getRightTagID(){
    if (hasLeftTarget()){
        double[] tag = getTableEntry("limelight-right", "tid").getDoubleArray(new double[6]);
        return tag[0];
    }
    return -1;
  }

  /**
   * Checks if right LimeLight has a target.
   *
   * @return True if LimeLight has a target, false otherwise.
   */
  public boolean hasRightTarget(){
    double hasTarget = getTableEntry("limelight-right", "tv").getDouble(0);
    return hasTarget == 1;
  }
  /**
   * Checks if left LimeLight has a target.
   *
   * @return True if LimeLight has a target, false otherwise.
   */
  public boolean hasLeftTarget(){
    double hasTarget = getTableEntry("limelight-left", "tv").getDouble(0);
    return hasTarget == 1;
  }
  /**
   * Checks if left LimeLight has a target.
   *
   * @return True if LimeLight has a target, false otherwise.
   */
  public boolean hasTarget(){
    return (hasLeftTarget() || hasRightTarget());
  }
  
  public static String poseAsString(Pose2d pose){
    return "Pose X: " + pose.getX() + " Pose Y: " + pose.getY() + " Rotation: " + pose.getRotation().getDegrees();
  }

  /**
   * Convert the 0 -> 180 -> -180 -> 0 rotation into 360 rotation (positive is counterclockwize)
   * 
   * @param rotation
   * @return double signifying the absolute rotation
   */
  public static double convertRotationToAbsolute(double rotation){
    if (rotation <= 0){
      rotation += 360;
    }
    return rotation;
  }

  /**
   * Convert the absolute rotation to the 180 rotation frc uses
   * 
   * @param rotation
   * @return double signifying the non absolute rotation
   */
  public static double convertRotationTo180(double rotation){
    if (rotation >= 180){
      rotation -= 360;
    }
    return rotation;
  }

  /**
   * Averages the positions of Pose2d objects.
   *
   * @param poseList A list of poses to average.
   * @return The average Pose2d.
  */
  private static Pose2d averagePoses(Pose2d[] poseList) {
    double xAverage = 0;
    double yAverage = 0;
    double yawAverage = 0;
    int length = poseList.length;
    for (Pose2d i: poseList){
      xAverage += i.getX();
      yAverage += i.getY();
      yawAverage += convertRotationToAbsolute(i.getRotation().getDegrees());
    }
    xAverage /= length;
    yAverage /= length;
    convertRotationTo180(yawAverage /= length);
    new Rotation2d();
    return new Pose2d(xAverage, yAverage, Rotation2d.fromDegrees(yawAverage));
  }

  /**
   * Gets the average of the current limelight poses
   * 
   * @return a pose2d object, empty pose2d if no limelight have targets
   */

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //limelightPose = getLimeLightAverage();

    SmartDashboard.putString("Limelight Average Pose: ", poseAsString(getLimeLightAverage()));
    SmartDashboard.updateValues();
  }
}
