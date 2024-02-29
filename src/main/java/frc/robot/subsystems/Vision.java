package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
//import org.photonvision.simulation.PhotonCameraSim;
//import org.photonvision.simulation.SimCameraProperties;
//import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class Vision {
    public final PhotonCamera camera;
    public final PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;

    // Simulation
    // private PhotonCameraSim cameraSim;
    // private VisionSystemSim visionSim;
    public AprilTagFieldLayout kTagLayout;
    SwerveSubsystem m_driveSubsystem;

    public Vision(SwerveSubsystem m_driveSubsystem) {
        this.m_driveSubsystem = m_driveSubsystem;
        camera = new PhotonCamera(kCameraName);

        try {
            kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        } catch (Exception e) {
            e.printStackTrace();
        }

        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    // /**
    //  * The latest estimated robot pose on the field from vision data. This may be empty. This should
    //  * only be called once per loop.
    //  *
    //  * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
    //  *     used for estimation.
    //  */
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    //     var visionEst = photonEstimator.update();
    //     double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    //     boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    //     /*if (SwerveDriveTelemetry.isSimulation) {
    //         visionEst.ifPresentOrElse(
    //                 est ->
    //                         getSimDebugField()
    //                                 .getObject("VisionEstimation")
    //                                 .setPose(est.estimatedPose.toPose2d()),
    //                 () -> {
    //                     if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
    //                 });
    //     }*/
    //     if (newResult) lastEstTimestamp = latestTimestamp;
    //     return visionEst;
    // }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        // utilize data from swerve to make it more accurate
        photonEstimator.setReferencePose(m_driveSubsystem.getPose());

        // now we update our vision :)
        var visionEst = photonEstimator.update();

        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult)
            lastEstTimestamp = latestTimestamp;
            
        return visionEst;
    }
    
    public int getAprilTagId(PhotonPipelineResult result) {
        return result.getBestTarget().getFiducialId();
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    /**
     * get distance to target
     * 
     * @param TARGET_HEIGHT_METERS target height in meters
     * @return if target height isnt there returns -1
     */
    public double getDistanceToTarget(double TARGET_HEIGHT_METERS) {
        double CAMERA_HEIGHT_METERS = Constants.Vision.kRobotToCam.getZ();
        // double TARGET_HEIGHT_METERS = 0;
        double CAMERA_PITCH_RADIANS = 0;

        var result = camera.getLatestResult();
        double range = -1;

        if (result.hasTargets()) {
            range = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
        }

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range
        // forwardSpeed = -controller.calculate(range, GOAL_RANGE_METERS);

        return range;
    }

    // // getting pitch to tag
    // public double getPitchToTag() {
    //     var estimatedPose = getEstimatedGlobalPose();
    //     // getting robot position
    //     Pose3d robotPose = getEstimatedGlobalPose().estimatedPose;
    //     // getting height of robot camera (placeholder for now)
    //     double camHeightMeters = 1.0;
    //     var result = camera.getLatestResult();
    //     var id = result.getBestTarget().getFiducialId();
    //     // getting the target's pose
    //     var targetPoseOptional = kTagLayout.getTagPose(id);
    //     Pose3d targetPose = targetPoseOptional.get();
    //     // getting distances
    //     double verticalDistance = targetPose.getZ() - camHeightMeters;
    //     double horizontalDistance = Math.hypot(targetPose.getX() - robotPose.getX(),
    //     targetPose.getY() - robotPose.getY());
    //     // finding the pitch radians
    //     double pitchRadians = Math.atan2(verticalDistance, horizontalDistance);
    //     return pitchRadians;
    // }

    /** A Field2d for visualizing our robot and objects on the field. */
    // public Field2d getSimDebugField() {
    // if (!SwerveDriveTelemetry.isSimulation) return null;
    // return visionSim.getDebugField();
    // }
}