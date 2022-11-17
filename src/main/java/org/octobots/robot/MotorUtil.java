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

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

/**
 * Utility methods relating to robot motor movement.
 *
 * @since 0.1.0
 */
public class MotorUtil {
    private MotorUtil() {
    }

    public static void setupMotionMagic(FeedbackDevice sensor, PIDConfig pidConfig,
                                        MotionMagicConfig mmConfig, BaseTalon... motors) {
        setupMotionMagic(sensor, pidConfig, 0, mmConfig, motors);
    }

    /**
     * <p>Configures Motion Magic motion profiling on given CTRE
     * Talon, Victor, or Falcon controlled motors.</p>
     *
     * <p>Uses the internal 1 kHz clock of the controller instead of the 20 ms
     * RoboRio clock. This is recommended as it removes the need to make
     * custom motion profiles, leading to faster turnaround times on subsystems.
     * It is recommended that this be the first motor configuration call
     * in any subsystem due to the ability for this to reset the controller
     * before making changes. This can be changed with {@code MotionMagicConfig}.</p>
     *
     * @param sensor the sensor attached to the controller used for loop feedback.
     * @param pidConfig the PIDF and range values to use on the controller.
     * @param slotIdx the PID profile slot this configuration applies to.
     * @param mmConfig the specific non-PID configuration options for this controller.
     * @param motors the motors for which Motion Magic is enabled on.
     *
     * @see PIDConfig
     * @see MotionMagicConfig
     * @since 0.1.0
     */
    public static void setupMotionMagic(FeedbackDevice sensor, PIDConfig pidConfig, int slotIdx,
                                        MotionMagicConfig mmConfig, BaseTalon... motors) {
        for (BaseTalon motor : motors) {
            if (mmConfig.doReset()) {
                motor.configFactoryDefault();
            }
            motor.selectProfileSlot(slotIdx, 0);
            motor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,
                    mmConfig.getPeriod(), mmConfig.getTimeout());
            motor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,
                    mmConfig.getPeriod(), mmConfig.getTimeout());
            if (sensor == FeedbackDevice.PulseWidthEncodedPosition
                    || sensor == FeedbackDevice.IntegratedSensor) {
                motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
                        mmConfig.getPeriod(), mmConfig.getTimeout());
            }
            for (int statusFrame : mmConfig.getStatusFrames()) {
                motor.setStatusFramePeriod(statusFrame, mmConfig.getPeriod(), mmConfig.getTimeout());
            }
            motor.configSelectedFeedbackSensor(sensor, 0, mmConfig.getTimeout());

            motor.configNominalOutputForward(0);
            motor.configNominalOutputReverse(0);
            motor.configPeakOutputForward(pidConfig.getRange());
            motor.configPeakOutputReverse(-pidConfig.getRange());

            motor.config_kP(slotIdx, pidConfig.getP());
            motor.config_kI(slotIdx, pidConfig.getI());
            motor.config_kD(slotIdx, pidConfig.getD());
            motor.config_kF(slotIdx, pidConfig.getF());

            if (mmConfig.getIntegralZone() != null) {
                motor.config_IntegralZone(slotIdx, mmConfig.getIntegralZone(), mmConfig.getTimeout());
            }
            if (mmConfig.getSCurveStrength() != null) {
                motor.configMotionSCurveStrength(mmConfig.getSCurveStrength(), mmConfig.getTimeout());
            }

            if (mmConfig.getMaxVel() != null) {
                motor.configMotionCruiseVelocity(mmConfig.getMaxVel());
            }
            if (mmConfig.getMaxAccel() != null) {
                motor.configMotionAcceleration(mmConfig.getMaxAccel());
            }
        }
    }

    /**
     * <p>Selects a sensor to use with a given CTRE motor controller.</p>
     *
     * <p>Overload for {@link #setupSelectedSensor(FeedbackDevice, int, boolean, BaseTalon...)}.</p>
     *
     * @see #setupSelectedSensor(FeedbackDevice, int, boolean, BaseTalon...)
     * @since 0.3.2
     */
    public static void setupSelectedSensor(FeedbackDevice sensor, BaseTalon... motors) {
        setupSelectedSensor(sensor, 20, true, motors);
    }

    /**
     * <p>Selects a sensor to use with a given CTRE motor controller.</p>
     *
     * <p>This approach allows the end user to use {@code motor.getSelectedSensorPosition()}
     * directly from the motor, which is faster than using the normal (for TalonSRX)
     * {@code motor.getSensorCollection().getPulseWidthPosition()}. This setup
     * only has to be performed once, then will allow repeated position get requests.</p>
     *
     * @param sensor the sensor device/type to use as feedback for this motor controller.
     * @param periodMs the period of the receipt status frame (typically 10-20ms).
     * @param motors all the motors to which this configuration will be applied.
     *
     * @since 0.3.2
     */
    public static void setupSelectedSensor(FeedbackDevice sensor, int periodMs,
                                           boolean setStatusFrame, BaseTalon... motors) {
        int statusFrame = 0;
        if (setStatusFrame) {
            switch (sensor) {
                case PulseWidthEncodedPosition:
                case CTRE_MagEncoder_Absolute:
                    statusFrame = StatusFrameEnhanced.Status_8_PulseWidth.value;
                    break;
                case IntegratedSensor:
                    // Status group ID for Talon FX integrated sensor
                    // https://docs.ctre-phoenix.com/en/stable/ch18_CommonAPI.html#motor-controllers
                    statusFrame = 21;
                    break;
                case QuadEncoder:
                case CTRE_MagEncoder_Relative:
                    statusFrame = StatusFrameEnhanced.Status_3_Quadrature.value;
                    break;
                default:
                    throw new UnsupportedOperationException("Unsupported feedback sensor");
            }
        }
        ErrorCode err;
        for (BaseTalon motor : motors) {
            if (setStatusFrame) {
                err = motor.setStatusFramePeriod(statusFrame, periodMs);
                checkError(err, false);
            }
            err = motor.configSelectedFeedbackSensor(sensor);
            checkError(err, false);
        }
    }

    /**
     * <p>Check an {@code ErrorCode} to see if it represents an error,
     * then trigger an {@code Error} if not running in silent mode.</p>
     *
     * <p>Will only return a value in the case of an error if the
     * silent mode boolean is set to true.</p>
     *
     * @param err the {@code ErrorCode} to check.
     * @param silent true if the {@code ErrorCode} should throw an {@code Error},
     *               (and run silently), else false.
     * @return true if the {@code ErrorCode} represents an error.
     *
     * @see #setupSelectedSensor(FeedbackDevice, int, boolean, BaseTalon...)
     * @since 0.3.2
     */
    private static boolean checkError(ErrorCode err, boolean silent) {
        boolean isError = err != ErrorCode.OK;
        if (isError && !silent) {
            throw new Error(err.name());
        }
        return isError;
    }

    /**
     * <p>Places limits on the range of CTRE Talon, Victor, or Falcon controlled motors.</p>
     *
     * <p>Limits are defined in ticks and apply to both power and positional control sets.
     * It is still recommended that both are limited manually if possible. Note that
     * this is a hard stop (despite being a soft limit) and does not account for velocity
     * accumulated while moving. This value does not persist after power-off.</p>
     *
     * @param forward the maximum ticks in the forward/positive direction.
     * @param reverse the minimum ticks in the reverse/backward/negative direction.
     * @param motors the motors to apply the soft limits onto.
     *
     * @since 0.1.0
     */
    public static void setSoftLimits(double forward, double reverse, BaseTalon... motors) {
        for (BaseTalon motor : motors) {
            motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
            motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen);
            motor.configForwardSoftLimitThreshold(forward);
            motor.configReverseSoftLimitThreshold(reverse);
            motor.configForwardSoftLimitEnable(true);
            motor.configReverseSoftLimitEnable(true);
        }
    }
}

