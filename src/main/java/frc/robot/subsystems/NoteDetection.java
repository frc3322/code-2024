package frc.robot.subsystems;

import java.io.IOException;
import java.util.Collections;
import java.util.Map;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NoteDetection {
    @SuppressWarnings("unchecked")
    public static Map<String, Object> parseJson(String jsonString) {
        try {
            // Create ObjectMapper instance
            ObjectMapper objectMapper = new ObjectMapper();

            // Convert JSON string to Map
            return objectMapper.readValue(jsonString, Map.class);
        } catch (IOException e) {
            e.printStackTrace();
            return Collections.emptyMap(); // Return null in case of any exception
        }
    }

    /**
     * Gets a network table entry from LimeLight.
     * @param entryKey The key of the entry to get.
     * @return The network table entry.
     */
    private NetworkTableEntry getTableEntry(String entryKey) {
        return NetworkTableInstance.getDefault().getEntry(entryKey);
    }
    
    /**
     * Get the pose of the closest note to the robot.
     * @return pose2d of the closest note to the robot if field space.
     */
    public Pose2d getClostestNote() {
        String[] jsonString = getTableEntry("notes").getStringArray(null);
        if (jsonString != null) {
            Map<String, Object> noteData = parseJson(jsonString[0]);
            return new Pose2d((double) noteData.get("x"), (double) noteData.get("y"), Rotation2d.fromDegrees((double) noteData.get("yaw")));
        } else {
            return null;
        }
    }

    /**
     * Get all notes in the field space.
     * @return List of all notes in the field space as pose2d's.
     */
    public Pose2d[] getAllNotes() {
        String[] jsonString = getTableEntry("notes").getStringArray(null);
        if (jsonString != null) {
            Pose2d[] notes = new Pose2d[jsonString.length];
            for (int i = 0; i < jsonString.length; i++) {
                Map<String, Object> noteData = parseJson(jsonString[i]);
                notes[i] = new Pose2d((double) noteData.get("x"), (double) noteData.get("y"), Rotation2d.fromDegrees((double) noteData.get("yaw")));
            }
            return notes;
        } else {
            return new Pose2d[0];
        }
    }
}
