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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.octobots.robot;
//package edu.wpi.first.wpilibj.examples.swervebot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
    private final Joystick joystick = new Joystick(1);
    private final DriveTrain swerve = new DriveTrain();

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
        driveWithJoystick(false);
    }

    private void driveWithJoystick(boolean fieldRelative) {
        //x move
        final var xSpeed =
                xLimiter.calculate(MathUtil.applyDeadband(joystick.getY(), 0.02))
                        * DriveTrain.MAX_SPEED;
        //y move
        final var ySpeed =
                yLimiter.calculate(MathUtil.applyDeadband(joystick.getY(), 0.02))
                        * DriveTrain.MAX_SPEED;
        //spiiiiiiiiiiiiiiiiin
        final var rot =
                rotLimiter.calculate(MathUtil.applyDeadband(joystick.getZ(), 0.02))
                        * DriveTrain.MAX_ANGULAR_SPEED;

        swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
}