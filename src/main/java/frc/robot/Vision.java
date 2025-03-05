package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class Vision {
  public static final int NO_TAG = 0;
  public static final Pose3d NO_TARGET = Pose3d.kZero;
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  private PhotonTrackedTarget nearestReefAprilTag;
  private Pose3d nearestReefAprilTagTransform = NO_TARGET;
  private int nearestTagId = NO_TAG;

  @Logged
  private Transform3d leftTransform = new Transform3d();
  @Logged
  private Transform3d rightTransform = new Transform3d();

  public final Trigger seesTag = new Trigger(this::seesTag);

  private final PhotonCamera right = new PhotonCamera("OV9281-1");
  private final PhotonCamera left = new PhotonCamera("OV9281-2");
  private final Set<Integer> reefIDs = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

  public void update() {
    var leftResults = left.getAllUnreadResults();
    var rightResults = right.getAllUnreadResults();

    // Find the ID number of the nearest reef AprilTag

    nearestTagId = leftResults.stream()
        .filter(lr -> rightResults.stream().anyMatch(rr -> rr.getBestTarget().getFiducialId() == lr.getBestTarget().getFiducialId()))
        .min(Comparator.comparingDouble(r -> getTransformRelativeToRobot(r.getBestTarget(), leftOffset).getTranslation().getNorm()))
        .map(r -> r.getBestTarget().getFiducialId())
        .orElse(NO_TAG);

    // Find the pose of the nearest reef AprilTag, with some debugging logging for the raw
    // transforms from the cameras

    PhotonTrackedTarget bestOverall = null;
    PhotonTrackedTarget bestLeft = getClosestTarget(leftResults, leftOffset);
    PhotonTrackedTarget bestRight = getClosestTarget(rightResults, rightOffset);

    if (bestLeft == null) {
      bestOverall = bestRight;
      if (bestOverall != null) {
        nearestReefAprilTagTransform = getTransformRelativeToRobot(bestOverall, rightOffset);
      } else {
        nearestReefAprilTagTransform = NO_TARGET;
      }
    } else if (bestRight == null) {
      bestOverall = bestLeft;
      nearestReefAprilTagTransform = getTransformRelativeToRobot(bestOverall, leftOffset);
    } else {
      if (getTransformRelativeToRobot(bestLeft, leftOffset).getTranslation().getNorm()
          < getTransformRelativeToRobot(bestRight, rightOffset).getTranslation().getNorm()) {
        bestOverall = bestLeft;
        nearestReefAprilTagTransform = getTransformRelativeToRobot(bestOverall, leftOffset);
      } else {
        bestOverall = bestRight;
        nearestReefAprilTagTransform = getTransformRelativeToRobot(bestOverall, rightOffset);
      }
    }
    nearestReefAprilTag = bestOverall;
    if (bestLeft == null) {
      leftTransform = new Transform3d();
    } else {
      leftTransform = bestLeft.bestCameraToTarget;
    }
    if (bestRight == null) {
      rightTransform = new Transform3d();
    } else {
      rightTransform = bestRight.bestCameraToTarget;
    }
  }

  /**
   * Gets the ID of the nearest detected AprilTag seen by both cameras. Returns {@link #NO_TAG} if
   * no AprilTag is detected by both cameras.
   */
  public int getNearestTagId() {
    return nearestTagId;
  }

  public boolean seesTag() {
    return nearestTagId != NO_TAG;
  }

  public Pose3d getRobotTransformNearestToReef() {
    return nearestReefAprilTagTransform;
  }

  public PhotonTrackedTarget getNearestReefAprilTag() {
    return nearestReefAprilTag;
  }

  private Pose3d getTransformRelativeToRobot(PhotonTrackedTarget target, Pose3d cameraPosition) {
    return cameraPosition.transformBy(target.bestCameraToTarget);
  }

  private PhotonTrackedTarget getClosestTarget(List<PhotonPipelineResult> pipelineResults, Pose3d cameraPosition) {
    return pipelineResults.stream()
        .flatMap(result -> result.getTargets().stream())
        .filter(target -> reefIDs.contains(target.getFiducialId()))
        .min(
            Comparator.comparingDouble(
                target -> getTransformRelativeToRobot(target, cameraPosition).getTranslation().getNorm()))
        .orElse(null);
  }
}
