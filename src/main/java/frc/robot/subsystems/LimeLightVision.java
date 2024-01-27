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
  
  public LimeLightVision() {
  }

  private static NetworkTableEntry getTableEntry(String limeLight, String entryKey) {
    return NetworkTableInstance.getDefault().getTable(limeLight).getEntry(entryKey);
  }
  /**
   * Gets the pose from the LimeLight vision system.
   *
   * @param limeLight The LimeLight table name.
   * @return The pose retrieved from LimeLight.
   */
  public static Pose2d getPose(String limeLight){
    double[] limeLightData = getTableEntry(limeLight, "botpose").getDoubleArray(new double[6]);
    return new Pose2d(limeLightData[0] + LimeLightConstants.fieldLengthOffset, limeLightData[1] + LimeLightConstants.fieldWidthOffset, Rotation2d.fromDegrees(limeLightData[5]));
  }

  /**
   * Gets the target ID from the LimeLight vision system.
   *
   * @param limeLight The LimeLight table name.
   * @return The target ID.
   */
  public double getTagID(String limeLight){
    if (hasTarget(limeLight)){
        double[] tag = getTableEntry(limeLight, "tid").getDoubleArray(new double[6]);
        return tag[0];
    }
    return -1;
  }

  /**
   * Checks if LimeLight has a target.
   *
   * @param limeLight The LimeLight table name.
   * @return True if LimeLight has a target, false otherwise.
   */
  public static boolean hasTarget(String limeLight){
    double hasTarget = getTableEntry(limeLight, "tv").getDouble(0);
    return hasTarget == 1;
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

  public static Pose2d getBestPose(String limeLightOne, String limeLightTwo, Pose2d driveTrainPose){
    boolean hasTargetOne = hasTarget(limeLightOne);
    boolean hasTargetTwo = hasTarget(limeLightTwo);

    SmartDashboard.putBoolean("LimeLight One Has Target", hasTargetOne);
    SmartDashboard.putBoolean("LimeLight Two Has Target", hasTargetTwo);
    if (hasTargetOne) {
      SmartDashboard.putString("LimeLight One Pose", poseAsString(getPose(limeLightOne)));
    }
    if (hasTargetTwo) {
      SmartDashboard.putString("LimeLight Two Pose", poseAsString(getPose(limeLightTwo)));
    }

    if (hasTargetOne && hasTargetTwo) {
      Pose2d limeLightOnePose = getPose(limeLightOne);
      Pose2d limeLightTwoPose = getPose(limeLightTwo);

      double limeLightOneDistance = driveTrainPose.getTranslation().getDistance(limeLightOnePose.getTranslation());
      double limeLightTwoDistance = driveTrainPose.getTranslation().getDistance(limeLightTwoPose.getTranslation());

      // if both are within the threshold average them
      if (limeLightOneDistance < LimeLightConstants.xyThreshold && limeLightTwoDistance < LimeLightConstants.xyThreshold) {
        Pose2d[] poseArray = {limeLightOnePose, limeLightTwoPose};
        SmartDashboard.putString("Lime Light Logic Status", "Both are within threshold");
        return averagePoses(poseArray);
      }
      if (limeLightOneDistance < LimeLightConstants.xyThreshold) {
        Pose2d[] poseArray = {limeLightOnePose, driveTrainPose};
        SmartDashboard.putString("Lime Light Logic Status", "Lime Light One is within threshold");
        return averagePoses(poseArray);
      }
      if (limeLightTwoDistance < LimeLightConstants.xyThreshold) {
        Pose2d[] poseArray = {limeLightTwoPose, driveTrainPose};
        SmartDashboard.putString("Lime Light Logic Status", "Lime Light Two is within threshold");
        return averagePoses(poseArray);
      }
      double limeOneTwoDistance = limeLightOnePose.getTranslation().getDistance(limeLightTwoPose.getTranslation());
      if (limeOneTwoDistance < LimeLightConstants.xyThreshold) {
        Pose2d[] poseArray = {limeLightOnePose, limeLightTwoPose};
        SmartDashboard.putString("Lime Light Logic Status", "Averaging only limelights");
        return averagePoses(poseArray);
      }
      SmartDashboard.putString("Lime Light Logic Status", "No limelights are within threshold, using drive train pose");
      return driveTrainPose;
    }
    else if (hasTargetOne) {
      Pose2d limeLightOnePose = getPose(limeLightOne);
      double limeLightOneDistance = driveTrainPose.getTranslation().getDistance(limeLightOnePose.getTranslation());
      if (limeLightOneDistance < LimeLightConstants.xyThreshold) {
        Pose2d[] poseArray = {limeLightOnePose, driveTrainPose};
        SmartDashboard.putString("Lime Light Logic Status", "Lime Light One is within threshold");
        return averagePoses(poseArray);
      }
      SmartDashboard.putString("Lime Light Logic Status", "Lime Light One is not within threshold, using drive train pose");
      return driveTrainPose;
    }
    else if (hasTargetTwo) {
      Pose2d limeLightTwoPose = getPose(limeLightTwo);
      double limeLightTwoDistance = driveTrainPose.getTranslation().getDistance(limeLightTwoPose.getTranslation());
      if (limeLightTwoDistance < LimeLightConstants.xyThreshold) {
        Pose2d[] poseArray = {limeLightTwoPose, driveTrainPose};
        SmartDashboard.putString("Lime Light Logic Status", "Lime Light Two is within threshold");
        return averagePoses(poseArray);
      }
      SmartDashboard.putString("Lime Light Logic Status", "Lime Light Two is not within threshold, using drive train pose");
      return driveTrainPose;
    }
    SmartDashboard.putString("Lime Light Logic Status", "No limelights have targets, using drive train pose");
    return driveTrainPose;
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
  public static Pose2d limeLightAverage(){
    if (hasTarget("limelight-right") && hasTarget("limelight-left")){
      Pose2d left = getPose("limelight-left");
      Pose2d right = getPose("limelight-right");
      Pose2d[] poseArray = {left, right};
      return averagePoses(poseArray);
    } else if (hasTarget("limelight-right")){
      Pose2d right = getPose("limelight-right");
      Pose2d[] poseArray = {right};
      return averagePoses(poseArray);
    }
    Pose2d left = getPose("limelight-left");
    Pose2d[] poseArray = {left};
    return averagePoses(poseArray);
    
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
