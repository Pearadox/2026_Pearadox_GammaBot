package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.CameraMountCalibration;
import frc.robot.subsystems.vision.CameraMountCalibration.Result;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Collects paired MegaTag1 observations from a stationary robot and reports the camera mount poses
 * that would make the two Limelights agree.
 *
 * <p>Run it disabled, with the robot parked where both cameras see tags (ideally the same tag).
 * Results are published under CamCal/ and the suggested mounts are formatted for typing into each
 * Limelight's web UI, which is where they belong: entering them there survives a reboot.
 *
 * <p>MegaTag1 only. MegaTag2 is seeded with the gyro's yaw, so it would hide exactly the yaw error
 * this is trying to find.
 */
public class CalibrateCameraMounts extends Command {
  private static final LoggedTunableNumber errorShareTowardCamera0 =
      new LoggedTunableNumber("CamCal/Error Share Toward Camera0", 0.5);
  private static final LoggedTunableNumber requiredSamples =
      new LoggedTunableNumber("CamCal/Required Samples", 50);
  private static final LoggedTunableNumber timeoutSeconds =
      new LoggedTunableNumber("CamCal/Timeout Seconds", 20);

  // Above these, the sample set is not trustworthy: the robot moved, or a camera is chewing on a
  // marginal tag.
  private static final double TRANSLATION_SPREAD_WARN_METERS = 0.01;
  private static final double ROTATION_SPREAD_WARN_DEGREES = 0.5;

  private final Vision vision;

  private final List<Pose3d> samples0 = new ArrayList<>();
  private final List<Pose3d> samples1 = new ArrayList<>();
  private final CameraMountCalibration.PoseAverager averager0 =
      new CameraMountCalibration.PoseAverager();
  private final CameraMountCalibration.PoseAverager averager1 =
      new CameraMountCalibration.PoseAverager();

  private int sharedTagSampleCount = 0;
  private double startTimestamp = 0.0;

  /** Last computed suggestions, so the separate apply button has something to push. */
  private static Result lastResult = null;

  public CalibrateCameraMounts(Vision vision) {
    this.vision = vision;
  }

  @Override
  public void initialize() {
    samples0.clear();
    samples1.clear();
    sharedTagSampleCount = 0;
    startTimestamp = Logger.getTimestamp() / 1.0e6;

    // Limelights are throttled hard while disabled, which is exactly when this runs.
    vision.unthrottleLimelights();

    Logger.recordOutput("CamCal/Status", "collecting");
  }

  /** The best MegaTag1 observation this loop, or null if there is nothing usable. */
  private PoseObservation latestUsableMegatag1(int cameraIndex) {
    PoseObservation best = null;

    for (PoseObservation observation : vision.getPoseObservations(cameraIndex)) {
      if (observation.type() != PoseObservationType.MEGATAG_1) continue;
      if (observation.tagCount() == 0) continue;
      if (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) continue;
      if (Math.abs(observation.pose().getZ()) > maxZError) continue;

      Pose3d pose = observation.pose();
      if (pose.getX() < 0.0
          || pose.getX() > aprilTagLayout.getFieldLength()
          || pose.getY() < 0.0
          || pose.getY() > aprilTagLayout.getFieldWidth()) {
        continue;
      }

      // Prefer the observation backed by the most tags.
      if (best == null || observation.tagCount() > best.tagCount()) {
        best = observation;
      }
    }

    return best;
  }

  private int countSharedTags() {
    Set<Integer> tags0 = new HashSet<>();
    for (int id : vision.getTagIds(0)) {
      tags0.add(id);
    }

    int shared = 0;
    for (int id : vision.getTagIds(1)) {
      if (tags0.contains(id)) shared++;
    }
    return shared;
  }

  @Override
  public void execute() {
    PoseObservation observation0 = latestUsableMegatag1(0);
    PoseObservation observation1 = latestUsableMegatag1(1);

    // Only same-loop pairs count. A sample from one camera taken while the other had nothing says
    // nothing about their disagreement.
    if (observation0 == null || observation1 == null) return;

    samples0.add(observation0.pose());
    samples1.add(observation1.pose());
    averager0.add(observation0.pose());
    averager1.add(observation1.pose());

    if (countSharedTags() > 0) sharedTagSampleCount++;

    Logger.recordOutput("CamCal/SampleCount", samples0.size());
  }

  @Override
  public boolean isFinished() {
    boolean haveEnough = samples0.size() >= (int) requiredSamples.get();
    boolean timedOut = (Logger.getTimestamp() / 1.0e6) - startTimestamp > timeoutSeconds.get();
    return haveEnough || timedOut;
  }

  @Override
  public void end(boolean interrupted) {
    vision.throttleLimelights();

    if (samples0.size() < 2) {
      Logger.recordOutput(
          "CamCal/Status",
          "FAILED: only "
              + samples0.size()
              + " paired samples. Park where both cameras see tags and try again.");
      return;
    }

    Pose3d mean0 = averager0.getMean();
    Pose3d mean1 = averager1.getMean();

    var spread0 = CameraMountCalibration.spread(samples0, mean0);
    var spread1 = CameraMountCalibration.spread(samples1, mean1);

    Pose3d mount0 =
        CameraMountCalibration.mountFromLimelightArray(vision.getCameraPoseRobotSpace(0));
    Pose3d mount1 =
        CameraMountCalibration.mountFromLimelightArray(vision.getCameraPoseRobotSpace(1));

    Result result =
        CameraMountCalibration.solve(mean0, mount0, mean1, mount1, errorShareTowardCamera0.get());
    lastResult = result;

    Transform3d disagreement = result.disagreement();
    double disagreementCm = disagreement.getTranslation().getNorm() * 100.0;
    double disagreementDeg = Units.radiansToDegrees(disagreement.getRotation().getAngle());

    Logger.recordOutput("CamCal/Samples", samples0.size());
    Logger.recordOutput("CamCal/SamplesWithSharedTag", sharedTagSampleCount);
    Logger.recordOutput("CamCal/Disagreement_cm", disagreementCm);
    Logger.recordOutput("CamCal/Disagreement_deg", disagreementDeg);
    Logger.recordOutput("CamCal/Camera0/Spread_cm", spread0.translationStdDevMeters() * 100.0);
    Logger.recordOutput("CamCal/Camera0/Spread_deg", spread0.rotationStdDevDegrees());
    Logger.recordOutput("CamCal/Camera1/Spread_cm", spread1.translationStdDevMeters() * 100.0);
    Logger.recordOutput("CamCal/Camera1/Spread_deg", spread1.rotationStdDevDegrees());

    logSuggestion(0, camera0Name, result.camera0());
    logSuggestion(1, camera1Name, result.camera1());

    List<String> warnings = new ArrayList<>();
    if (sharedTagSampleCount == 0) {
      warnings.add(
          "no sample had a tag both cameras could see, so field layout error leaks into this");
    }
    if (spread0.translationStdDevMeters() > TRANSLATION_SPREAD_WARN_METERS
        || spread1.translationStdDevMeters() > TRANSLATION_SPREAD_WARN_METERS) {
      warnings.add("translation spread over 1 cm: did the robot move?");
    }
    if (spread0.rotationStdDevDegrees() > ROTATION_SPREAD_WARN_DEGREES
        || spread1.rotationStdDevDegrees() > ROTATION_SPREAD_WARN_DEGREES) {
      warnings.add("rotation spread over 0.5 deg: marginal tags or a moving robot");
    }

    Logger.recordOutput("CamCal/Warnings", String.join("; ", warnings));
    Logger.recordOutput(
        "CamCal/Status",
        String.format(
            "done: %d samples, cameras disagree by %.1f cm / %.2f deg%s",
            samples0.size(),
            disagreementCm,
            disagreementDeg,
            warnings.isEmpty() ? "" : " (see CamCal/Warnings)"));
  }

  private static void logSuggestion(
      int index, String cameraName, CameraMountCalibration.MountSuggestion suggestion) {
    String prefix = "CamCal/Camera" + index + "/";
    Transform3d correction = suggestion.correction();

    Logger.recordOutput(prefix + "Name", cameraName);
    Logger.recordOutput(
        prefix + "CurrentMount",
        Arrays.toString(CameraMountCalibration.mountToLimelightArray(suggestion.currentMount())));
    Logger.recordOutput(
        prefix + "SuggestedMountArray",
        CameraMountCalibration.mountToLimelightArray(suggestion.suggestedMount()));
    Logger.recordOutput(
        prefix + "SuggestedMount",
        CameraMountCalibration.formatForWebUI(suggestion.suggestedMount()));
    Logger.recordOutput(prefix + "Correction_cm", correction.getTranslation().getNorm() * 100.0);
    Logger.recordOutput(
        prefix + "Correction_deg", Units.radiansToDegrees(correction.getRotation().getAngle()));
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  /**
   * Pushes the last suggestion to both cameras over NetworkTables. This is a try-it-now button, not
   * a commit: a Limelight forgets an overridden mount pose when it reboots, so a bad apply heals
   * itself. Type the numbers into the web UI to make them stick.
   */
  public static Command applyLastSuggestion(Vision vision) {
    return Commands.runOnce(
            () -> {
              if (lastResult == null) {
                Logger.recordOutput("CamCal/Status", "nothing to apply: run the calibration first");
                return;
              }

              vision.setCameraPoseRobotSpace(
                  0,
                  CameraMountCalibration.mountToLimelightArray(
                      lastResult.camera0().suggestedMount()));
              vision.setCameraPoseRobotSpace(
                  1,
                  CameraMountCalibration.mountToLimelightArray(
                      lastResult.camera1().suggestedMount()));

              Logger.recordOutput(
                  "CamCal/Status", "applied to both cameras (reverts when they reboot)");
            })
        .ignoringDisable(true);
  }
}
