package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.LimelightHelpers.PoseEstimate;

/**
 * Vision subsystem — reads MegaTag2 pose estimates from the Limelight 4
 * and feeds them into the drivetrain's pose estimator every loop.
 *
 * How to use:
 *   1. Instantiate this in RobotContainer.
 *   2. Call updatePoseEstimation() inside this subsystem's periodic() — already done below.
 *   3. Pass your drivetrain reference in via the constructor.
 *
 * Tuning tips:
 *   - Lower std-dev numbers = trust vision MORE (less filtering).
 *   - Raise them if your pose jumps around a lot.
 *   - We reject estimates with fewer than 1 tag or an avg tag distance > 4 m.
 */
public class Vision extends SubsystemBase {

    // ── Change this if your Limelight has a different hostname ────────────────
    private static final String LIMELIGHT_NAME = "limelight";

    // ── Reject measurements with avg tag distance beyond this (meters) ────────
    private static final double MAX_TAG_DISTANCE_M = 4.0;

    // ── Standard deviations for vision pose measurements ─────────────────────
    //    [x (m), y (m), theta (rad)]
    //    We trust x/y from MegaTag2 but ignore its yaw (set theta huge).
    private static final Matrix<N3, N1> VISION_STD_DEVS =
        VecBuilder.fill(0.5, 0.5, 9999999);

    private final CommandSwerveDrivetrain drivetrain;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        updatePoseEstimation();
    }

    /**
     * Sends the robot's current yaw to the Limelight (required for MegaTag2),
     * then reads the pose estimate and feeds it into the drivetrain if valid.
     */
    private void updatePoseEstimation() {
        // MegaTag2 requires the robot's current heading to compute a pose.
        // We feed the drivetrain's best-known yaw each loop.
        double currentYawDeg = drivetrain.getState().Pose.getRotation().getDegrees();

        LimelightHelpers.SetRobotOrientation(
            LIMELIGHT_NAME,
            currentYawDeg,
            0.0, 0.0,   // yaw rate, pitch
            0.0, 0.0,0.0    // pitch rate, roll
        );

        // Read MegaTag2 pose (always field-relative, blue-alliance origin)
        PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        // Reject bad estimates
        if (!isValidEstimate(estimate)) {
            SmartDashboard.putBoolean("Vision/HasTarget", false);
            return;
        }

        SmartDashboard.putBoolean("Vision/HasTarget", true);
        SmartDashboard.putNumber("Vision/TagCount",       estimate.tagCount);
        SmartDashboard.putNumber("Vision/AvgTagDist (m)", estimate.avgTagDist);
        SmartDashboard.putString("Vision/Pose",           estimate.pose.toString());

        // Feed the measurement into the drivetrain's Kalman filter
        drivetrain.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            VISION_STD_DEVS
        );
    }

    /**
     * Returns true if the estimate is non-null, has at least one tag,
     * and the average tag distance is within our threshold.
     */
    private boolean isValidEstimate(PoseEstimate estimate) {
        if (estimate == null)                          return false;
        if (estimate.tagCount < 1)                     return false;
        if (estimate.avgTagDist > MAX_TAG_DISTANCE_M)  return false;
        return true;
    }
}
