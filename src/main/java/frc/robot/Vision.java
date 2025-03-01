package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Comparator;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

@Logged
public class Vision {
  private PhotonTrackedTarget nearestReefAprilTag;
  private Transform3d nearestReefAprilTagTransform;

  private Transform3d leftTransform = new Transform3d();
  private Transform3d rightTransform = new Transform3d();

  private final PhotonCamera right = new PhotonCamera("OV9281-1");
  private final PhotonCamera left = new PhotonCamera("OV9281-2");
  private final Set<Integer> reefIDs = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

  public void update() {
    PhotonTrackedTarget bestOverall = null;
    PhotonTrackedTarget bestLeft = getClosestTarget(left, leftOffset);
    PhotonTrackedTarget bestRight = getClosestTarget(right, rightOffset);

    if (bestLeft == null) {
      bestOverall = bestRight;
      if (bestOverall != null) {
        nearestReefAprilTagTransform = getTransformRelativeToRobot(bestOverall, rightOffset);
      } else {
        nearestReefAprilTagTransform = null;
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
  public Transform3d getRobotTransformNearestToReef() {
    return nearestReefAprilTagTransform;
  }

  public PhotonTrackedTarget getNearestReefAprilTag() {
    return nearestReefAprilTag;
  }

  private Transform3d getTransformRelativeToRobot(PhotonTrackedTarget target, Transform3d offset) {
    return target.bestCameraToTarget.plus(offset.inverse());
  }

  private PhotonTrackedTarget getClosestTarget(PhotonCamera camera, Transform3d offset) {
    return camera.getAllUnreadResults().stream()
        .flatMap(result -> result.getTargets().stream())
        .filter(target -> reefIDs.contains(target.getFiducialId()))
        .min(
            Comparator.comparingDouble(
                target -> getTransformRelativeToRobot(target, offset).getTranslation().getNorm()))
        .orElse(null);
  }
}
