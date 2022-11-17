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

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import java.util.ArrayList;


public class SwerveModule {

    private static final double WHEEL_RADIUS = 0.03915;
    private static final int ENCODER_RESOLUTION = 4096;
    private static final double STEER_MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION; // radians
    private static final double GEARING = 11.0 / 40.0;
    private static final double DRIVE_MOTOR_TICK_TO_SPEED = 10 * GEARING * (2 * Math.PI * WHEEL_RADIUS) / 2048; // m/s
    // Controller Constants
    private static final double MAX_TURN_ACCELERATION = 20000; // Rad/s
    private static final double MAX_TURN_VELOCITY = 20000; // Rad/s
    private static final int TIMEOUT_MS = 60;
    private static final MotionMagicConfig TM_MM_CONFIG = new MotionMagicConfig(
            new ArrayList<>(), true,
            MAX_TURN_VELOCITY, MAX_TURN_ACCELERATION,
            600, 0,
            TIMEOUT_MS, 10
    );
    private static final PIDConfig TM_MM_PID = new PIDConfig(3.4, 0.01, 0, 0);

    // Drive Motor Motion Magic
    private static final MotionMagicConfig DM_MM_CONFIG = new MotionMagicConfig(
            new ArrayList<>(), true,
            10000.0, 10000.0,
            300, 2,
            TIMEOUT_MS, 10
    );
    private static final PIDConfig DM_MM_PID = new PIDConfig(0.035, 0.0001, 0, 0.06);
    private static final double kModuleMaxAngularVelocity = DriveTrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration =
            2 * Math.PI; // radians per second squared
    private final double zeroTicks;
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonSRX steeringMotor;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     */
    public SwerveModule(int driveMotorChannel, int steeringMotorChannel, double absZeroTicks) {


        // Steer Motor
        this.steeringMotor = new WPI_TalonSRX(steeringMotorChannel);
        TM_MM_PID.setTolerance(0);
        MotorUtil.setupMotionMagic(FeedbackDevice.PulseWidthEncodedPosition, TM_MM_PID, TM_MM_CONFIG, steeringMotor);
        steeringMotor.setSensorPhase(false);
        steeringMotor.setInverted(true);
        steeringMotor.setNeutralMode(NeutralMode.Coast);

        // Drive Motor
        this.driveMotor = new WPI_TalonFX(driveMotorChannel, "can1");
        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor, DM_MM_PID, DM_MM_CONFIG, driveMotor);
        driveMotor.configAllowableClosedloopError(0, 5);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setStatusFramePeriod(21, 10);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        // Current Limits
        this.driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 30, 0.05)); //How much current the motor can use (outputwise)
        this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 33, 33, 0.05)); //How much current can be supplied to the motor

        this.steeringMotor.enableCurrentLimit(true);
        this.steeringMotor.configPeakCurrentDuration(10);
        this.steeringMotor.configContinuousCurrentLimit(20);
        this.steeringMotor.configPeakCurrentLimit(21);

        this.zeroTicks = steeringMotor.getSelectedSensorPosition() + 2 * absZeroTicks;

        try {
            Thread.sleep(200);
        } catch (Exception e) {
            // Ignore all sleep exceptions
        }
    }


    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
//    public SwerveModuleState getPosition() {
//        return new SwerveModuleState(
//                m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.get()));
//    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
//        var optimizedAngle = desiredState.optimize(desiredState, );
        driveMotor.set(ControlMode.Velocity, desiredState.speedMetersPerSecond);
        steeringMotor.set(ControlMode.MotionMagic, desiredState.angle.getRadians()/(2*Math.PI)*ENCODER_RESOLUTION);
    }
}