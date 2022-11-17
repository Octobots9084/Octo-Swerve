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

/**
 * <p>Stores a PIDF configuration through loop gain constants.</p>
 *
 * <p>Terms may be accessed or changed through standard getters/setters.
 * Contains helper method to apply terms to a given CTRE motor controller.
 * Should be used in all places where PIDF loops are needed.</p>
 *
 * @since 0.1.0
 */
public class PIDConfig {
    protected double kP;
    protected double kI;
    protected double kD;
    protected double kF;
    protected double tolerance;
    protected double range;

    /**
     * Constructs a PID configuration with the specified loop gain constants.
     *
     * @param kP proportional gain; output proportional to current error.
     * @param kI integral gain; output based on accumulated error to exponentiate kP.
     * @param kD derivative gain; typically used for damping.
     * @param kF feed forward constant.
     * @param range maximum (negated to get minimum) motor movement
     *              percentage allowed for PIDF loop to set [-1, 1].
     *
     * @since 0.1.0
     */
    public PIDConfig(double kP, double kI, double kD, double kF, double range) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.range = range;
    }

    /**
     * <p>Constructs a PID configuration with the specified loop gain constants.
     * Has a default range value of [-1, 1] or full range.</p>
     *
     * <p>Overload for {@link #PIDConfig(double, double, double, double, double)}.</p>
     *
     * @see #PIDConfig(double, double, double, double, double)
     * @since 0.1.0
     */
    public PIDConfig(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, 1.0);
    }

    /**
     * <p>Constructs a PID configuration with the specified loop gain constants.
     * Has a default range value of [-1, 1] or full range and no feed forward.</p>
     *
     * <p>Overload for {@link #PIDConfig(double, double, double, double, double)}.</p>
     *
     * @see #PIDConfig(double, double, double, double, double)
     * @since 0.1.0
     */
    public PIDConfig(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0, 1.0);
    }

    public double getP() {
        return kP;
    }

    public double getI() {
        return kI;
    }

    public double getD() {
        return kD;
    }

    public double getF() {
        return kF;
    }

    public double getTolerance() {
        return tolerance;
    }

    public double getRange() {
        return range;
    }

    public PIDConfig setP(double kP) {
        this.kP = kP;
        return this;
    }

    public PIDConfig setI(double kI) {
        this.kI = kI;
        return this;
    }

    public PIDConfig setD(double kD) {
        this.kD = kD;
        return this;
    }

    public PIDConfig setF(double kF) {
        this.kF = kF;
        return this;
    }

    public PIDConfig setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public PIDConfig setRange(double pidRange) {
        this.range = pidRange;
        return this;
    }
}
