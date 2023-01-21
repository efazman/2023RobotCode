// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax m_neoMotor;
  public Intake() {
    m_neoMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_NEO_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    
  }
}
