package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;
    private final StructPublisher<Pose2d> visionPosePub;

    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;

    public VisionSubsystem() {
        this.camera = new PhotonCamera(FRONT_CAMERA_NAME);

        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        } catch (Exception e) {
            layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        }
        this.fieldLayout = layout;

        this.poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                ROBOT_TO_CAMERA);
        this.visionPosePub = NetworkTableInstance.getDefault()
                .getStructTopic("Vision/EstimatedPose", Pose2d.struct).publish();

        if (RobotBase.isSimulation()) {
            setupSimulation();
        }
    }

    private void setupSimulation() {
        visionSim = new VisionSystemSim("VisionWorld");
        visionSim.addAprilTags(fieldLayout);

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        props.setFPS(30);
        props.setAvgLatencyMs(35);

        cameraSim = new PhotonCameraSim(camera, props);
        visionSim.addCamera(cameraSim, ROBOT_TO_CAMERA);

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
    }

    public void updateSimPose(Pose2d robotPose) {
        if (RobotBase.isSimulation() && visionSim != null) {
            visionSim.update(robotPose);
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) return Optional.empty();
        return poseEstimator.update(result);
    }

    /**
     * Gets the yaw of a specific AprilTag.
     *
     * @param tagId The ID of the tag to find.
     * @return The yaw of the target, or an empty Optional if the tag is not visible.
     */
    public Optional<Double> getTargetYaw(int tagId) {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (target.getFiducialId() == tagId) {
                    return Optional.of(target.getYaw());
                }
            }
        }
        return Optional.empty();
    }

    /**
     * Gets the yaw of the first visible AprilTag from a list of valid tag IDs.
     * Useful for alliance-aware hub targeting.
     *
     * @param validTagIds Array of tag IDs to search for (e.g., BLUE_HUB_TAGS or RED_HUB_TAGS)
     * @return The yaw of the first visible target tag, or an empty Optional if none are visible.
     */
    public Optional<Double> getTargetYawFromList(int[] validTagIds) {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                // Check if this target's ID is in our valid list
                for (int validId : validTagIds) {
                    if (target.getFiducialId() == validId) {
                        return Optional.of(target.getYaw());
                    }
                }
            }
        }
        return Optional.empty();
    }

    @Override
    public void periodic() {
        getEstimatedGlobalPose().ifPresent(est -> {
            visionPosePub.set(est.estimatedPose.toPose2d());
        });
    }
}