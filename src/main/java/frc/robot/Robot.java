// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot {
  private final CANSparkMax m_leftMotor = new CANSparkMax(0, MotorType.kBrushed);
  private final CANSparkMax m_rightMotor = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_leftMotor2 = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_rightMotor2 = new CANSparkMax(3, MotorType.kBrushed);
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftMotor, m_leftMotor2);
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightMotor, m_rightMotor2);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  private final WPI_TalonFX m_shooter = new WPI_TalonFX(8);
  private final WPI_TalonFX m_elevator = new WPI_TalonFX(9);

  private final CANSparkMax m_intake = new CANSparkMax(10, MotorType.kBrushed);

  private final DoubleSolenoid m_reach = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  private final DoubleSolenoid m_extend = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with tank drive.
    // That means that the Y axis of the left stick moves the left side
    // of the robot forward and backward, and the Y axis of the right stick
    // moves the right side of the robot forward and backward.
    m_robotDrive.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightY());
     
    if (m_operatorController.getXButton()) {
      m_elevator.set(0.5); 
    }
    else {
      m_elevator.set(0);
    }

  }
}
