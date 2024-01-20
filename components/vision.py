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

from components.chassis import Chassis
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
        chassis: Chassis,
    ) -> None:
        self.mid = 615
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
        if not (results := self.camera.getLatestResult()).getTargets():
            return

        # if we have already processed these results
        timestamp = results.getTimestamp()
        if timestamp == self.last_timestamp and wpilib.RobotBase.isReal():
            return
        self.last_timestamp = timestamp

        # old results cause pose estimator to crash and aren't very useful anyway
        # if abs(wpilib.Timer.getFPGATimestamp() - timestamp) > 0.5:
        #    return # IT FAILS HERE

        # cs = [i.x for i in target.getDetectedCorners()]
        # mid = ((cs[0] + cs[1])/2+(cs[2]+cs[3])/2)/2
        # self.mid = mid

        if results.multiTagResult.estimatedPose.isPresent():
            p = results.multiTagResult.estimatedPose
            pose, reprojectionErr = choose_pose(
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

            p.best, p.alt, self.last_pose_z = poses
            pose = choose_pose(
                p.best,
                p.alt,
                self.chassis.get_pose(),
            )

            # filter out likely bad targets
            if target.getPoseAmbiguity() > 0.25:
                continue

            self.field_pos_obj.setPose(pose[0])
            self.chassis.estimator.addVisionMeasurement(pose[0], results.getTimestamp())
            change = (
                self.chassis.get_pose().translation().distance(pose[0].translation())
            )
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


def get_target_skew(target: PhotonTrackedTarget):
    tag_to_cam = target.getBestCameraToTarget().inverse()
    return math.atan2(tag_to_cam.y, tag_to_cam.x)


def choose_pose(
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
