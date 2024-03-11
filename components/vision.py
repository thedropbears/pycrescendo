import math
import time
from typing import Optional

import wpilib
import wpiutil.log
from magicbot import tunable
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from wpimath.geometry import Pose2d, Rotation3d, Transform3d, Translation3d, Pose3d

from components.chassis import ChassisComponent
from utilities.game import apriltag_layout


class VisualLocalizer:
    """
    This localizes the robot from AprilTags on the field,
    using information from a single PhotonVision camera.
    """

    # Give bias to the best pose by multiplying this const to the alt dist
    BEST_POSE_BIAS = 1.2

    # Time since the last target sighting we allow before informing drivers
    TIMEOUT = 1.0  # s

    add_to_estimator = tunable(True)
    should_log = tunable(False)

    last_pose_z = tunable(0.0, writeDefault=False)

    def __init__(
        self,
        # The name of the camera in PhotonVision.
        name: str,
        # Position of the camera relative to the center of the robot
        pos: Translation3d,
        # The camera rotation.
        rot: Rotation3d,
        field: wpilib.Field2d,
        data_log: wpiutil.log.DataLog,
        chassis: ChassisComponent,
    ) -> None:
        self.camera = PhotonCamera(name)
        self.camera_to_robot = Transform3d(pos, rot).inverse()
        self.last_timestamp = -1
        self.last_recieved_timestep = -1.0

        self.single_best_log = field.getObject(name + "single_best_log")
        self.single_alt_log = field.getObject(name + "single_alt_log")
        self.multi_best_log = field.getObject(name + "multi_best_log")
        self.multi_alt_log = field.getObject(name + "multi_alt_log")
        self.field_pos_obj = field.getObject(name + "vision_pose")
        self.pose_log_entry = wpiutil.log.FloatArrayLogEntry(
            data_log, name + "vision_pose"
        )

        self.chassis = chassis

    def execute(self) -> None:
        # stop warnings in simulation
        if wpilib.RobotBase.isSimulation():
            return

        results = self.camera.getLatestResult()
        # if results didn't see any targets
        if not results.getTargets():
            return

        # if we have already processed these results
        timestamp = results.getTimestamp()

        if timestamp == self.last_timestamp:
            return
        self.last_recieved_timestep = time.monotonic()
        self.last_timestamp = timestamp

        if results.multiTagResult.estimatedPose.isPresent:
            p = results.multiTagResult.estimatedPose
            pose = (Pose3d() + p.best + self.camera_to_robot).toPose2d()
            reprojectionErr = p.bestReprojError

            self.field_pos_obj.setPose(pose)

            if self.add_to_estimator:
                self.chassis.estimator.addVisionMeasurement(
                    pose,
                    timestamp,
                    (reprojectionErr * 1.5, reprojectionErr * 1.5, reprojectionErr / 2),
                )

            if self.should_log:
                self.multi_best_log.setPose(
                    Pose2d(
                        p.best.x,
                        p.best.y,
                        p.best.rotation().toRotation2d().radians(),
                    )
                )
                self.multi_alt_log.setPose(
                    Pose2d(
                        p.alt.x,
                        p.alt.y,
                        p.alt.rotation().toRotation2d().radians(),
                    )
                )
        else:
            for target in results.getTargets():
                # filter out likely bad targets
                if target.getPoseAmbiguity() > 0.25:
                    continue

                poses = estimate_poses_from_apriltag(self.camera_to_robot, target)
                if poses is None:
                    # tag doesn't exist
                    continue

                best, alt, self.last_pose_z = poses
                pose = choose_pose(
                    best,
                    alt,
                    self.chassis.get_pose(),
                )

                self.field_pos_obj.setPose(pose)
                self.chassis.estimator.addVisionMeasurement(pose, timestamp)

                if self.should_log:
                    self.single_best_log.setPose(
                        Pose2d(
                            target.bestCameraToTarget.x,
                            target.bestCameraToTarget.y,
                            target.bestCameraToTarget.rotation()
                            .toRotation2d()
                            .radians(),
                        )
                    )
                    self.single_alt_log.setPose(
                        Pose2d(
                            target.altCameraToTarget.x,
                            target.altCameraToTarget.y,
                            target.altCameraToTarget.rotation()
                            .toRotation2d()
                            .radians(),
                        )
                    )

    def sees_target(self):
        return time.monotonic() - self.last_recieved_timestep < self.TIMEOUT


def estimate_poses_from_apriltag(
    cam_to_robot: Transform3d, target: PhotonTrackedTarget
) -> Optional[tuple[Pose2d, Pose2d, float]]:
    tag_id = target.getFiducialId()
    tag_pose = apriltag_layout.getTagPose(tag_id)
    if tag_pose is None:
        return None

    best_pose = tag_pose.transformBy(
        target.getBestCameraToTarget().inverse()
    ).transformBy(cam_to_robot)
    alternate_pose = (
        tag_pose.transformBy(target.getAlternateCameraToTarget().inverse())
        .transformBy(cam_to_robot)
        .toPose2d()
    )
    return best_pose.toPose2d(), alternate_pose, best_pose.z


def get_target_skew(target: PhotonTrackedTarget) -> float:
    tag_to_cam = target.getBestCameraToTarget().inverse()
    return math.atan2(tag_to_cam.y, tag_to_cam.x)


def choose_pose(best_pose: Pose2d, alternate_pose: Pose2d, cur_robot: Pose2d) -> Pose2d:
    """Picks either the best or alternate pose estimate"""
    best_dist = best_pose.translation().distance(cur_robot.translation())
    alternate_dist = (
        alternate_pose.translation().distance(cur_robot.translation())
        * VisualLocalizer.BEST_POSE_BIAS
    )

    if best_dist < alternate_dist:
        return best_pose
    else:
        return alternate_pose
