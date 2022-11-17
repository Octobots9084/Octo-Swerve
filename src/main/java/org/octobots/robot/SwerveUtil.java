/*
 * This file is part of GradleRIO-Redux-example, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Octobots <https://github.com/Octobots9084>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.octobots.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.octobots.robot.MathUtil;

public class SwerveUtil {
    public static double clampAngle(double angle) {
        double high = Math.PI;
        angle = MathUtil.wrapToCircle(angle, 2 * Math.PI);
        if (angle > high) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static double getAngleDiff(double src, double target) {
        double diff = target - src;
        if (Math.abs(diff) <= Math.PI) {
            return diff;
        }

        if (diff > 0) {
            diff -= 2 * Math.PI;
        } else {
            diff += 2 * Math.PI;
        }

        return diff;
    }

    public static double getClosestAngle(double clampedAngle, double target) {
        double diff;
        double positiveTarget = target + Math.PI;
        double negativeTarget = target - Math.PI;
        if (Math.abs(getAngleDiff(clampedAngle, positiveTarget)) <= Math.abs(getAngleDiff(clampedAngle, negativeTarget))) {
            diff = getAngleDiff(clampedAngle, positiveTarget);
        } else {
            diff = getAngleDiff(clampedAngle, negativeTarget);
        }

        if (Math.abs(getAngleDiff(clampedAngle, target)) <= Math.abs(diff)) {
            diff = getAngleDiff(clampedAngle, target);
        }

        return diff;
    }

    public static SwerveModuleState optimizeSwerveStates(SwerveModuleState state, double moduleAngle) {
        double clampedAng = clampAngle(moduleAngle);
        double targetRotation = state.angle.getRadians();
        double diff = getClosestAngle(clampedAng, targetRotation);
        double targetAng = moduleAngle + diff;

        double targetSpeed = state.speedMetersPerSecond;
        if (!MathUtil.isWithinTolerance(targetRotation, clampAngle(targetAng), 0.1)) {
            targetSpeed *= -1;
        }

        return new SwerveModuleState(targetSpeed, new Rotation2d(targetAng));
    }

    private SwerveUtil() {
    }
}
