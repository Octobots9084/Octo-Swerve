// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.octobots.robot;
//package edu.wpi.first.wpilibj.examples.swervebot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
    private final Joystick joystick = new Joystick(1);
    private final DriveTrain m_swerve = new DriveTrain();

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
        driveWithJoystick(true);
    }

    private void driveWithJoystick(boolean fieldRelative) {
        //x move
        final var xSpeed =
                m_xspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getY(), 0.02))
                        * DriveTrain.kMaxSpeed;
        //y move
        final var ySpeed =
                m_yspeedLimiter.calculate(MathUtil.applyDeadband(joystick.getY(), 0.02))
                        * DriveTrain.kMaxSpeed;
        //speen
        final var rot =
                m_rotLimiter.calculate(MathUtil.applyDeadband(joystick.getZ(), 0.02))
                        * DriveTrain.kMaxAngularSpeed;

        m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    }
}