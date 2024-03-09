package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimeLightConstants;

public class LimeLight {
    private String limeLightID;

    /**
     * Creates a new LimeLight object.
     * @param limeLightID The ID of the LimeLight. ex. "limelight-left"
     */
    public LimeLight(String limeLightID){
        this.limeLightID = limeLightID;
    }

    /**
     * Gets a network table entry from LimeLight.
     * @param entryKey The key of the entry to get.
     * @return The network table entry.
     */
    private NetworkTableEntry getTableEntry(String entryKey) {
        return NetworkTableInstance.getDefault().getTable(this.limeLightID).getEntry(entryKey);
    }

    /**
     * Set a network table entry to the LimeLight.
     * @param entryKey The key of the entry to set.
     * @param value The value to set the entry to.
     */
    private void setTableEntry(String entryKey, double value) {
        NetworkTableInstance.getDefault().getTable(this.limeLightID).getEntry(entryKey).setDouble(value);
    }

    /**
     * Gets the field pose from the LimeLight.
     * @return The pose retrieved from LimeLight.
     */
    public Pose2d getPose(){
        double[] limeLightData = getTableEntry("botpose").getDoubleArray(new double[6]);
        return new Pose2d(limeLightData[0] + LimeLightConstants.fieldLengthOffset, limeLightData[1] + LimeLightConstants.fieldWidthOffset, Rotation2d.fromDegrees(limeLightData[5]));
    }

    /**
     * Checks if the LimeLight has a target.
     * @return Whether the LimeLight has a target.
     */
    public boolean hasTarget(){
        return getTableEntry("tv").getDouble(0) == 1;
    }

    /**
     * Returns the current pose as a string.
     * @return The current pose as a string.
     */
    public String poseAsString(Pose2d pose){
        return "Pose X: " + pose.getX() + " Pose Y: " + pose.getY() + " Rotation: " + pose.getRotation().getDegrees();
    }
    public String poseAsString(){
        Pose2d pose = getPose();
        return "Pose X: " + pose.getX() + " Pose Y: " + pose.getY() + " Rotation: " + pose.getRotation().getDegrees();
    }

    /**
     * Gets the target ID of the detected tag.
     * @return The target ID of the detected tag.
     */
    public int getTagId(){
        if (hasTarget()){
            return (int)getTableEntry("tv").getDouble(-1);
        }
        return -1;
    }

    /**
     * Gets the total latency of the system.
     * @return The total latency of the system.
     */
    public double getTotalLatency(){
        return getTableEntry("tl").getDouble(0) + getTableEntry("cl").getDouble(0);
    }

    /**
     * Gets the limelight id of this object
     * @return The limelight id of this object
     */
    public String getLimeLightID(){
        return this.limeLightID;
    }

    /**
     * Sets the LED of the limelight to on
     */
    public void setOn(){
        setTableEntry("ledMode", 3);
    }

    /**
     * Sets the LED of the limelight to blink
     */
    public void setBlink(){
        setTableEntry("ledMode", 2);
    }

    /**
     * Sets the LED of the limelight to off
     */
    public void setOff(){
        setTableEntry("ledMode", 1);
    }

    /**
     * Sets the LED of the limelight to pipeline default
     */
    public void setPipeline(){
        setTableEntry("pipeline", 0);
    }
}
