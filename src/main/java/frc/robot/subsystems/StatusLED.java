// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.trobot5013lib.led.SolidColorPattern;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;

public class StatusLED extends SubsystemBase {
   private TrobotAddressableLED m_AddressableLED = new TrobotAddressableLED(9, 39);

  /** Creates a new StatusLED. */
  public StatusLED() {
    
  }

  @Override
  public void periodic() {
    // check conditions and set patterns
    m_AddressableLED.setPattern(new SolidColorPattern(Color.kRed));

    m_AddressableLED.update();
  }
}
