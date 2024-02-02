/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

object Robot : TimedRobot() {
  private var autonomousCommand: Command? = null

  private val robotContainer = RobotContainer

  override fun robotInit() {}

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()
  }

  override fun disabledInit() {}

  override fun disabledPeriodic() {}

  override fun autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand()

    autonomousCommand?.schedule()
  }

  override fun autonomousPeriodic() {}

  override fun teleopInit() {
    autonomousCommand?.cancel()
  }

  override fun teleopPeriodic() {}

  override fun testInit() {
    CommandScheduler.getInstance().cancelAll()
  }

  override fun testPeriodic() {}

  override fun simulationInit() {}

  override fun simulationPeriodic() {}
}
