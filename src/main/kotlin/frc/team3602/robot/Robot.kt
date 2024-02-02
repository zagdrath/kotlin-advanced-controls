/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

class Robot : TimedRobot() {
  private var autonomousCommand: Command? = null

  private final val robotContainer = RobotContainer()

  override public fun robotInit() {}

  override public fun robotPeriodic() {
    CommandScheduler.getInstance().run()
  }

  override public fun disabledInit() {}

  override public fun disabledPeriodic() {}

  override public fun autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand()

    autonomousCommand?.schedule()
  }

  override public fun autonomousPeriodic() {}

  override public fun teleopInit() {
    autonomousCommand?.cancel()
  }

  override public fun teleopPeriodic() {}

  override public fun testInit() {
    CommandScheduler.getInstance().cancelAll()
  }

  override public fun testPeriodic() {}

  override public fun simulationInit() {}

  override public fun simulationPeriodic() {}
}
