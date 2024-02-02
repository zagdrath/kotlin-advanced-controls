/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

import frc.team3602.robot.constants.operatorinterfaceconstants.*
import frc.team3602.robot.subsystems.PivotSubsystem

object RobotContainer {
  // Subsystems
  private val pivotSubsys = PivotSubsystem

  // Operator interfaces
  private val xboxController = CommandXboxController(kXboxControllerPort)

  // Autonomous
  private val sendableChooser = SendableChooser<Command>()

  init {
    configDefaultCommands()
    configButtonBindings()
    configAutonomous()
  }

  private fun configDefaultCommands() {
    pivotSubsys.defaultCommand = pivotSubsys.holdAngle()
  }

  private fun configButtonBindings() {}

  private fun configAutonomous() {
    SmartDashboard.putData(sendableChooser)
  }

  fun getAutonomousCommand(): Command {
    return sendableChooser.selected
  }
}
