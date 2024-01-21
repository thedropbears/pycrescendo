import math
import typing
from typing import Optional

import wpilib
import wpiutil.log
from magicbot import tunable
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from photonlibpy.multiTargetPNPResult import PNPResult
from wpimath.geometry import Pose2d, Rotation3d, Transform3d, Translation3d, Pose3d

from components.chassis import ChassisComponent
from utilities.game import apriltag_layout


class VisualLocalizer:
    """
    This localizes the robot from AprilTags on the field,
    using information from a single PhotonVision camera.
    """

    add_to_estimator = tunable(False)
    should_log = tunable(False)

    rejected_in_row = tunable(0.0)
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

        self.field_pos_obj = field.getObject("vision_pose")
        self.pose_log_entry = wpiutil.log.DoubleArrayLogEntry(data_log, "vision_pose")

        self.chassis = chassis

    def on_disable(self) -> None:
        self.add_to_estimator = False

    def on_enable(self) -> None:
        self.add_to_estimator = True

    def execute(self) -> None:
        # stop warnings in simulation
        if wpilib.RobotBase.isSimulation():
            return
        # if results didn't see any targets

        results = self.camera.getLatestResult()
        if not results.getTargets():
            return

        # if we have already processed these results
        timestamp = results.getTimestamp()
        if timestamp == self.last_timestamp and wpilib.RobotBase.isReal():
            return
        self.last_timestamp = timestamp

        # TODO Figure out the latency issue
        # if abs(wpilib.Timer.getFPGATimestamp() - timestamp) > 0.5:
        #    return # IT FAILS HERE

        if results.multiTagResult.estimatedPose.isPresent:
            p = results.multiTagResult.estimatedPose
            pose, reprojectionErr = choose_pose_multi(
                p, self.camera_to_robot, self.chassis.get_pose()
            )

            self.field_pos_obj.setPose(pose)

            if self.add_to_estimator:
                self.chassis.estimator.addVisionMeasurement(
                    pose,
                    timestamp,
                    (reprojectionErr, reprojectionErr, reprojectionErr / 3),
                )

            if self.should_log:
                ground_truth_pose = self.chassis.get_pose()
                trans_error1: float = ground_truth_pose.translation().distance(
                    p.best.translation()
                )
                trans_error2: float = ground_truth_pose.translation().distance(
                    p.alt.translation()
                )
                rot_error1: float = (  # type: ignore
                    ground_truth_pose.rotation() - p.best.rotation()
                ).radians()
                rot_error2: float = (  # type: ignore
                    ground_truth_pose.rotation() - p.alt.rotation()
                ).radians()

                self.pose_log_entry.append(
                    [
                        p.best.x,
                        p.best.y,
                        typing.cast(float, p.best.rotation().radians()),
                        trans_error1,  # error of main pose
                        rot_error1,
                        p.alt.x,
                        p.alt.y,
                        typing.cast(float, p.alt.rotation().radians()),
                        trans_error2,
                        rot_error2,
                        ground_truth_pose.x,
                        ground_truth_pose.y,
                    ]
                )

        for target in results.getTargets():
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

            # filter out likely bad targets
            if target.getPoseAmbiguity() > 0.25:
                continue

            self.field_pos_obj.setPose(pose)
            self.chassis.estimator.addVisionMeasurement(pose, results.getTimestamp())
            change = self.chassis.get_pose().translation().distance(pose.translation())
            if change > 1.0:
                self.rejected_in_row += 1
                if self.rejected_in_row < 10:
                    continue
            else:
                self.rejected_in_row //= 2


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


def choose_pose_multi(
    estimated_pose: PNPResult, cam_to_robot: Transform3d, cur_pos: Pose2d
) -> tuple[Pose2d, float]:
    """Picks either the best or alternate pose estimate"""
    p = (Pose3d() + estimated_pose.best + cam_to_robot).toPose2d()
    best_dist = p.translation().distance(cur_pos.translation())
    best_preferance = 1.2
    p2 = (Pose3d() + estimated_pose.alt + cam_to_robot).toPose2d()
    alt_dist = p2.translation().distance(cur_pos.translation()) * best_preferance

    if best_dist < alt_dist:
        return p, estimated_pose.bestReprojError
    else:
        return p2, estimated_pose.altReprojError


def choose_pose(best_pose: Pose2d, alternate_pose: Pose2d, cur_robot: Pose2d) -> Pose2d:
    """Picks either the best or alternate pose estimate"""
    best_dist = best_pose.translation().distance(cur_robot.translation())
    best_preferance = 1.2
    alternate_dist = (
        alternate_pose.translation().distance(cur_robot.translation()) * best_preferance
    )

    if best_dist < alternate_dist:
        return best_pose
    else:
        return alternate_pose
