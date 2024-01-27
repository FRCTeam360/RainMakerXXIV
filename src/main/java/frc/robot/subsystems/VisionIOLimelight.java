<<<<<<< Updated upstream
package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOLimelight implements VisionIO {
    private String name;
    private final Transform3d cameraOffset;
    private double[] lastData = new double[6];

    private double pastTimestamp;

    /**
     * Implements Limelight camera
     *
     * @param name Name of the camera.
     * @param cameraOffset Location of the camera on the robot (from center, positive x towards the arm, positive y to the left, and positive angle is counterclockwise.
     */
    public VisionIOLimelight(String name, Transform3d cameraOffset) {
        this.name = name;
        this.cameraOffset = cameraOffset;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        NetworkTableEntry botposeEntry;
        double[] data;
        // decides the pose based on the alliance
        if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
            botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue");
        } else if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
            botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpired");
        } else {
            botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose");
        }
        data = botposeEntry.getDoubleArray(new double[7]);
        long updateTime = botposeEntry.getLastChange();
        Pose2d pose = new Pose2d(data[0], data[1], new Rotation2d(Math.toRadians(data[5]))); // .transformBy(cameraOffset);

        // set if the Limelight has a target
        if (NetworkTableInstance.getDefault().getTable(name).getEntry("tv").getDouble(0) == 1) {
            inputs.hasTarget = true;
        } else {
            inputs.hasTarget = false;
        }

        // calculates total latency
        double latency = data[6] / 1000;

        if (inputs.hasTarget) {
            // sets inputs
            inputs.timestamp = Timer.getFPGATimestamp() - latency;

            //            inputs.isNew = !Arrays.equals(data, lastData);

            inputs.isNew = true;

            inputs.x = pose.getX();
            inputs.y = pose.getY();
            inputs.rotation = pose.getRotation().getRadians();
        } else {
            inputs.isNew = false;
        }

        lastData = data;
    }

    @Override
    public String getName() {
        return name;
    }
}
=======
// package frc.robot.subsystems;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;

// public class VisionIOLimelight implements VisionIO {
//     private String name;
//     private final Transform3d cameraOffset;
//     private double[] lastData = new double[6];

//     private double pastTimestamp;

//     /**
//      * Implements Limelight camera
//      *
//      * @param name Name of the camera.
//      * @param cameraOffset Location of the camera on the robot (from center, positive x towards the arm, positive y to the left, and positive angle is counterclockwise.
//      */
//     public VisionIOLimelight(String name, Transform3d cameraOffset) {
//         this.name = name;
//         this.cameraOffset = cameraOffset;
//     }

//     @Override
//     public void updateInputs(VisionIOInputs inputs) {
//         NetworkTableEntry botposeEntry;
//         double[] data;
//         // decides the pose based on the alliance
//         if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
//             botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue");
//         } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
//             botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpired");
//         } else {
//             botposeEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose");
//         }
//         data = botposeEntry.getDoubleArray(new double[7]);
//         long updateTime = botposeEntry.getLastChange();
//         Pose2d pose = new Pose2d(data[0], data[1], new Rotation2d(Math.toRadians(data[5]))); // .transformBy(cameraOffset);

//         // set if the Limelight has a target
//         if (NetworkTableInstance.getDefault().getTable(name).getEntry("tv").getDouble(0) == 1) {
//             inputs.hasTarget = true;
//         } else {
//             inputs.hasTarget = false;
//         }

//         // calculates total latency
//         double latency = data[6] / 1000;

//         if (inputs.hasTarget) {
//             // sets inputs
//             inputs.timestamp = Timer.getFPGATimestamp() - latency;

//             //            inputs.isNew = !Arrays.equals(data, lastData);

//             inputs.isNew = true;

//             inputs.x = pose.getX();
//             inputs.y = pose.getY();
//             inputs.rotation = pose.getRotation().getRadians();
//         } else {
//             inputs.isNew = false;
//         }

//         lastData = data;
//     }

//     @Override
//     public String getName() {
//         return name;
//     }
// }
>>>>>>> Stashed changes
