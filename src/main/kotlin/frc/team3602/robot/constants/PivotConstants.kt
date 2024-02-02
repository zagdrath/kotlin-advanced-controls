/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.constants.pivotconstants

import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage

const val kPivotMotorId = 2
const val kPivotFollowerId = 3

const val kPivotMotorCurrentLimit = 40
const val kPivotFollowerCurrentLimit = 40

const val kPivotPositionConversionFactor = 360.0

// Linear system model
const val kMomentOfIntertia = 2.0
const val kGearing = (240.0 / 1.0)

// LQR tolerances
val kPosTolerance: Measure<Angle> = Degrees.of(0.25)
val kVelTolerance: Measure<Angle> = Degrees.of(10.0)

val kControlEffort: Measure<Voltage> = Volts.of(12.0)

// Kalman filter deviations
val kModelPosDeviation: Measure<Angle> = Degrees.of(10.0)
val kModelVelDeviation: Measure<Angle> = Degrees.of(20.0)
val kEncoderPosDeviation: Measure<Angle> = Degrees.of(0.25)

val kMaxVoltage: Measure<Voltage> = Volts.of(12.0)

// Trapezoid profile
val kMaxVelocity: Measure<Velocity<Angle>> = DegreesPerSecond.of(100.0)
val kMaxAcceleration: Measure<Velocity<Velocity<Angle>>> = DegreesPerSecond.per(Second).of(75.0)

val kConstraints = TrapezoidProfile.Constraints(kMaxVelocity.`in`(RadiansPerSecond), kMaxAcceleration.`in`(RadiansPerSecond.per(Second)))

// Feedforward gains
const val kS = 0.35
