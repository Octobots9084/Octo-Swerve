package org.octobots.robot.util;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import org.octobots.robot.swerve.DriveTrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import com.pathplanner.lib.PathPlannerTrajectory;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Objects;

public class TrajectoryFollower {
    public static Trajectory getTrajectoryFromPathweaver(String path) {
        String trajectoryJSON = "paths/" + path + ".wpilib.json";
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        try {
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Pathweaver path not found", e.getStackTrace());
        }
        return null;
    }

    private final HolonomicDriveController holonomicDriveController;
    private final PathPlannerTrajectory pathPlannerTrajectory;
    private final double startTime;
    private final PoseEstimator estimator;
    private final Trajectory trajectory;
    private final Gyro gyro;
    private final DriveTrain driveTrain;
    private final boolean useAngle;

    public TrajectoryFollower(Trajectory trajectory, boolean resetPose, Gyro gyro, DriveTrain driveTrain) {
        this(trajectory, null, resetPose, gyro, driveTrain, true);
    }

    public TrajectoryFollower(PathPlannerTrajectory trajectory, boolean resetPose, Gyro gyro, DriveTrain driveTrain) {
        this(null, trajectory, resetPose, gyro, driveTrain, true);
    }

    private TrajectoryFollower(Trajectory trajectory, PathPlannerTrajectory pathPlannerTrajectory,
                               boolean resetPose, Gyro gyro, DriveTrain driveTrain, boolean useAngle) {
        this.holonomicDriveController = driveTrain.getHolonomicDriveController();
        this.pathPlannerTrajectory = pathPlannerTrajectory;
        this.startTime = Timer.getFPGATimestamp();
        this.gyro = gyro;
        this.driveTrain = driveTrain;
        this.useAngle = useAngle;
        this.estimator = driveTrain.getPoseEstimator();
        this.trajectory = trajectory;
        if (trajectory == null && pathPlannerTrajectory == null) {
            DriverStation.reportError("Trying to drive null path", false);
            return;
        }
        if (resetPose) {
            estimator.resetPose(getInitialPose());
        }
    }

    private Pose2d getInitialPose() {
        if (trajectory != null) {
            return new Pose2d(trajectory.getInitialPose().getTranslation(), gyro.getRotation2d());
        } else {
            return new Pose2d(pathPlannerTrajectory.getInitialPose().getTranslation(), gyro.getRotation2d());
        }
    }

    public boolean isFinished() {
        if (trajectory == null && pathPlannerTrajectory == null) {
            return true;
        }

        return Timer.getFPGATimestamp() - this.startTime > Objects.requireNonNullElse(trajectory, this.pathPlannerTrajectory).getTotalTimeSeconds();
    }

    /**
     * Call this method periodically to follow a trajectory.
     */
    public void followController() {
        if (trajectory == null && pathPlannerTrajectory == null) {
            return;
        }

        double timePassed = Timer.getFPGATimestamp() - this.startTime;

        var controls = trajectory == null ? followPathPlannerTrajectory(timePassed) : followTrajectory(timePassed);

        driveTrain.drive(controls.vxMetersPerSecond, controls.vyMetersPerSecond, useAngle ? DriveTrain.getInstance().getRotationSpeed() : 0, false);
    }

    private ChassisSpeeds followPathPlannerTrajectory(double intTime) {
        var plannerState = (PathPlannerTrajectory.PathPlannerState) pathPlannerTrajectory.sample(intTime);

        DriveTrain.getInstance().setTargetRotationAngle(plannerState.holonomicRotation.getDegrees());
        return holonomicDriveController.calculate(
                estimator.getRobotPose(),
                plannerState,
                plannerState.holonomicRotation
        );
    }

    private ChassisSpeeds followTrajectory(double intTime) {
        DriveTrain.getInstance().setTargetRotationAngle(estimator.getRobotPose().getRotation().getDegrees());
        return holonomicDriveController.calculate(
                estimator.getRobotPose(),
                trajectory.sample(intTime),
                estimator.getRobotPose().getRotation()
        );
    }
}