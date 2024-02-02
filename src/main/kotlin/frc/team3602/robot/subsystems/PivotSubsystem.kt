/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems

import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkAbsoluteEncoder.Type

import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.Command

import frc.team3602.robot.constants.robotconstants.*
import frc.team3602.robot.constants.pivotconstants.*
import java.util.function.Supplier

import kotlin.math.sign

object PivotSubsystem : Subsystem {
  // Motors
  private val pivotMotor = CANSparkMax(kPivotMotorId, MotorType.kBrushless)
  private val pivotFollower = CANSparkMax(kPivotFollowerId, MotorType.kBrushless)

  // Encoders
  private val pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle)

  // Controls
  private val plant = LinearSystemId.createSingleJointedArmSystem(
          DCMotor.getNEO(2),
          kMomentOfIntertia,
          kGearing
  )

  private val controller = LinearQuadraticRegulator(
          plant,
          VecBuilder.fill(kPosTolerance.`in`(Radians), kVelTolerance.`in`(Radians)),
          VecBuilder.fill(kControlEffort.`in`(Volts)),
          kLoopTime.`in`(Seconds)
  )

  private val observer = KalmanFilter(
          Nat.N2(),
          Nat.N1(),
          plant,
          VecBuilder.fill(kModelPosDeviation.`in`(Radians), kModelVelDeviation.`in`(Radians)),
          VecBuilder.fill(kEncoderPosDeviation.`in`(Radians)),
          kLoopTime.`in`(Seconds)
  )

  private val loop = LinearSystemLoop(
          plant,
          controller,
          observer,
          kMaxVoltage.`in`(Volts),
          kLoopTime.`in`(Seconds)
  )

  private val positionSupplier: Supplier<Measure<Angle>> = Supplier { getPosition() }
  private val velocitySupplier: Supplier<Measure<Velocity<Angle>>> = Supplier { getVelocity() }

  private var lastProfileReference = TrapezoidProfile.State(getPosition(), getVelocity())

  private val profile = TrapezoidProfile(kConstraints)

  init {
    loop.reset(VecBuilder.fill(getPosition().`in`(Radians), getVelocity().`in`(RadiansPerSecond)))

    configPivotSubsys()
  }

  private fun getPosition(): Measure<Angle> {
    return Degrees.of(pivotEncoder.position)
  }

  private fun getVelocity(): Measure<Velocity<Angle>> {
    return DegreesPerSecond.of(pivotEncoder.velocity)
  }

  fun moveToAngle(goal: Measure<Angle>) {
    lastProfileReference = profile.calculate(
            kLoopTime.`in`(Seconds),
            lastProfileReference,
            TrapezoidProfile.State(goal.`in`(Radians), 0.0)
    )

    loop.setNextR(lastProfileReference.position, lastProfileReference.velocity)
    loop.correct(VecBuilder.fill(positionSupplier.get().`in`(Radians)))
    loop.predict(kLoopTime.`in`(Seconds))

    pivotMotor.setVoltage(loop.getU(0) + sign(lastProfileReference.velocity) * kS)
  }

  fun holdAngle(): Command {
    return run {
      loop.setNextR(lastProfileReference.position, lastProfileReference.velocity)
      loop.correct(VecBuilder.fill(positionSupplier.get().`in`(Radians)))
      loop.predict(kLoopTime.`in`(Seconds))

      pivotMotor.setVoltage(loop.getU(0))
    }
  }

  fun stop(): Command {
    return runOnce { pivotMotor.stopMotor() }
  }

  private fun configPivotSubsys() {
    // Pivot motor config
    pivotMotor.setIdleMode(IdleMode.kBrake)
    pivotMotor.setSmartCurrentLimit(kPivotMotorCurrentLimit)
    pivotMotor.enableVoltageCompensation(pivotMotor.busVoltage)

    // Pivot follower config
    pivotFollower.setIdleMode(IdleMode.kBrake)
    pivotFollower.follow(pivotMotor, true)
    pivotFollower.setSmartCurrentLimit(kPivotFollowerCurrentLimit)
    pivotFollower.enableVoltageCompensation(pivotFollower.busVoltage)

    // Pivot encoder config
    pivotEncoder.setPositionConversionFactor(kPivotPositionConversionFactor)

    pivotMotor.burnFlash()
    pivotFollower.burnFlash()
  }
}
