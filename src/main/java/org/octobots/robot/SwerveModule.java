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
    private static final double kWheelRadius = 0.0508;
    private static final int kEncoderResolution = 4096;
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

    private final WPI_TalonFX driveMotor;
    private final WPI_TalonSRX steeringMotor;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController =
            new ProfiledPIDController(
                    1,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(
                            kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     */
    public SwerveModule(int driveMotorChannel, int steeringMotorChannel, double zeroTicks) {
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
        this.driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50 * 0.4, 50 * 0.4, 0.05)); //How much current the motor can use (outputwise)
        this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 53 * 0.4, 53 * 0.4, 0.05)); //How much current can be supplied to the motor

        this.steeringMotor.enableCurrentLimit(true);
        this.steeringMotor.configPeakCurrentDuration(0);
        this.steeringMotor.configContinuousCurrentLimit(20);
        this.steeringMotor.configPeakCurrentLimit(21);

        try {
            Thread.sleep(200);
        } catch (Exception e) {
            // Ignore all sleep exceptions
        }
    }

    public double convertVelocityToTicksPer100ms(double velocity) {
        return velocity / DRIVE_MOTOR_TICK_TO_SPEED;
    }

    public void setDriveMotorVelocity(double metersPerSecond) {
        //shuffleboard.setEntry("target velocity " + driveMotor.getDeviceID(), metersPerSecond);
        driveMotor.set(TalonFXControlMode.Velocity, convertVelocityToTicksPer100ms(metersPerSecond));
    }

    public void setSteeringMotorAngle(double angleInRad) {
        //shuffleboard.setEntry("target Angle" + driveMotor.getDeviceID(), angleInRad);
        steeringMotor.set(ControlMode.Position, angleInRad);
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
        setDriveMotorVelocity(desiredState.speedMetersPerSecond);
        setSteeringMotorAngle(desiredState.angle.getRadians());
    }
}