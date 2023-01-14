/*
 * This file is part of OctoSwerve-Revamp, licensed under the GNU General Public License (GPLv3).
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

package org.octobots.robot.swerve;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAlternateEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.octobots.robot.MotorIDs;
import org.octobots.robot.util.*;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class SwerveModule {
    // Physical Constants
    private static final double WHEEL_RADIUS = 0.03915;
    private static final int ENCODER_RESOLUTION = 4096;
    private static final double GEARING = 11.0 / 40.0;
    private static final double GEARING_TURN_MOTORS = 1.0 / 20.0;
    private static final double DRIVE_MOTOR_TICK_TO_SPEED = 10 * GEARING * (2 * Math.PI * WHEEL_RADIUS) / 2048; // m/s
    private static final double STEER_MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION / GEARING_TURN_MOTORS; // radians
    // Controller Constants
    private static final double MAX_TURN_ACCELERATION = 20000; // Rad/s
    private static final double MAX_TURN_VELOCITY = 20000; // Rad/s
    private static final double MIN_TURN_VELOCITY = 18000; // Rad/s
    private static final double ALLOWED_CLOSED_LOOP_ERROR = 0.1;
    private static final int TIMEOUT_MS = 60;

    // Turn Motor Smart Motion
    private static final SmartMotionConfig TM_SM_CONFIG = new SmartMotionConfig(
            true,
            MAX_TURN_VELOCITY, MIN_TURN_VELOCITY, MAX_TURN_ACCELERATION, ALLOWED_CLOSED_LOOP_ERROR
    );
    private static final PIDConfig TM_SM_PID = new PIDConfig(0.15, 0.001, 0.005, 0);

    // Drive Motor Motion Magic
    private static final MotionMagicConfig DM_MM_CONFIG = new MotionMagicConfig(
            new ArrayList<>(), true,
            10000.0, 10000.0,
            300, 2,
            TIMEOUT_MS, 10
    );
    private static final PIDConfig DM_MM_PID = new PIDConfig(0.035, 0.0001, 0, 0.06);

    private final double zeroTicks;

    // Motors
    private final WPI_TalonFX driveMotor;
    private final CANSparkMax steeringMotor;

    // Thread-Safe angles to reduce CAN usage
    private final AtomicReference<Double> swerveAngle = new AtomicReference<>(0.0);
    private final AtomicReference<Double> swerveSpeed = new AtomicReference<>(0.0);

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel    ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     * @param zeroTicks            ticks when angle = 0
     */
    public SwerveModule(int driveMotorChannel, int steeringMotorChannel, double zeroTicks) {
        this.zeroTicks = zeroTicks;

        // Steer Motor
        this.steeringMotor = new CANSparkMax(steeringMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        TM_SM_PID.setTolerance(0);
        this.steeringMotor.restoreFactoryDefaults();
        SparkMaxEncoderType steeringMotorEncoderType = SparkMaxEncoderType.relative;
        MotorUtil.setupSmartMotion(steeringMotorEncoderType, TM_SM_PID, TM_SM_CONFIG, SparkMaxAlternateEncoder.Type.kQuadrature,ENCODER_RESOLUTION, steeringMotor);


        // Drive Motor
        this.driveMotor = new WPI_TalonFX(driveMotorChannel, MotorIDs.CANFD_NAME);
        MotorUtil.setupMotionMagic(FeedbackDevice.IntegratedSensor, DM_MM_PID, DM_MM_CONFIG, driveMotor);
        driveMotor.configAllowableClosedloopError(0, 5);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        driveMotor.setStatusFramePeriod(21, 10);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        StatusFrameDemolisher.demolishStatusFrames(driveMotor, false);

        // Current Limits
        this.driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 0.05)); //How much current the motor can use (outputwise)
        this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 53, 53, 0.05)); //How much current can be supplied to the motor

        this.steeringMotor.setSmartCurrentLimit(21, 20);

        try {
            Thread.sleep(200);
        } catch (Exception e) {
            // Ignore all sleep exceptions
        }
    }

    public double getAbsoluteAngle() {
        return SwerveUtil.clampAngle(getAngle());
    }

    public double getAngle() {
        return swerveAngle.get();
    }

    public double convertAngleToTick(double angleInRads) {
        return (angleInRads / STEER_MOTOR_TICK_TO_ANGLE) + zeroTicks;
    }

    public double convertVelocityToTicksPer100ms(double velocity) {
        return velocity / DRIVE_MOTOR_TICK_TO_SPEED;
    }

    public double getPosTicks() {
        return steeringMotor.getEncoder().getPosition();
    }

    public double getDriveTicks() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return swerveSpeed.get();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(SwerveUtil.clampAngle(getAngle())));
    }

    public void setDriveMotorVelocity(double metersPerSecond) {
        driveMotor.set(TalonFXControlMode.Velocity, convertVelocityToTicksPer100ms(metersPerSecond));
    }

    public void setSteeringMotorAngle(double angleInRad) {
        steeringMotor.getPIDController().setReference(angleInRad, CANSparkMax.ControlType.kPosition);
    }

    public void updateSwerveInformation() {
        swerveAngle.set((steeringMotor.getEncoder().getPosition() - zeroTicks) * STEER_MOTOR_TICK_TO_ANGLE);
        swerveSpeed.set(driveMotor.getSensorCollection().getIntegratedSensorVelocity() * DRIVE_MOTOR_TICK_TO_SPEED);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        // Optimize the swerve state and set it
        var optimizedAngle = SwerveUtil.optimizeSwerveStates(state, getAngle());
        setDriveMotorVelocity(optimizedAngle.speedMetersPerSecond);
        setSteeringMotorAngle(convertAngleToTick(optimizedAngle.angle.getRadians()));
    }
}
