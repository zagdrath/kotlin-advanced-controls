/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot

import edu.wpi.first.wpilibj.RobotBase

object Main { 
  @JvmStatic
  fun main(args: Array<String>) {
    RobotBase.startRobot { Robot() }
  }
}
