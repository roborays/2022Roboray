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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  private final DoubleSolenoid m_reach = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
  private final DoubleSolenoid m_extend = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 4);

  // Speed Constants
  private final double intakeSpeed = 0.5;
  private final double elevatorSpeed = 0.5;
  private final double shooterSpeed = 0.5;

  // Auto Time Contants
  private Timer timer;
  private double step1Time = 2;
  private double step2Time = step1Time + 3;
  private double step3Time = step2Time + 3;
  private double step4Time = step3Time + 4;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    timer = new Timer();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // This will run by default.
        // Only one of the statements will run
        if (timer.get() <= step1Time) {
          // Do nothing so we don't get in someone else's way.
        } else if (timer.get() <= step2Time) {
          // Run the elevator and the shooter
          m_elevator.set(elevatorSpeed);
          m_shooter.set(shooterSpeed);
        } else if (timer.get() <= step3Time) {
          // Stop the elevator and the shooter.  Drive backwards
          m_elevator.set(0);
          m_shooter.set(0);
          m_robotDrive.arcadeDrive(-0.25, 0);
        } else if (timer.get() <= step4Time) {
          // Stop the drivetrain
          m_robotDrive.arcadeDrive(0, 0);
        }
        break;
    }
  }

  @Override
  public void teleopPeriodic() {
    /* DRIVER CONTROLS */
    // Drive with tank drive.
    // That means that the Y axis of the left stick moves the left side
    // of the robot forward and backward, and the Y axis of the right stick
    // moves the right side of the robot forward and backward.
    m_robotDrive.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightY());

    // This uses 2 buttons to extend and retract a cylinder
    if (m_driverController.getLeftBumperPressed()) {
      m_reach.set(Value.kForward);
    }
    if (m_driverController.getRightBumperPressed()) {
      m_reach.set(Value.kReverse);
    }

    // Here we are using the triggers like buttons.  I don't particularly recommend this.
    if (m_driverController.getLeftTriggerAxis() > 0.5 && m_extend.get().equals(Value.kReverse)) {
      m_extend.set(Value.kForward);
    }
    if (m_driverController.getRightTriggerAxis() > 0.5 && m_extend.get().equals(Value.kForward)) {
      m_extend.set(Value.kReverse);
    }
    
    /* OPERATOR CONTROLS */
    // This is an example of a button that needs to be held
    if (m_operatorController.getXButton()) {
      m_elevator.set(elevatorSpeed); 
    }
    else {
      m_elevator.set(0);
    }

    // A & B buttons turn the shooter on and off
    if (m_operatorController.getAButtonPressed()) {
      m_shooter.set(shooterSpeed);
    }
    if (m_operatorController.getBButtonPressed()) {
      m_shooter.set(0);
    }

    //  Left and Right Bumpers turn the intake on and off
    if (m_operatorController.getLeftBumperPressed()) {
      m_intake.set(intakeSpeed);
    }
    if (m_operatorController.getRightBumperPressed()) {
      m_shooter.set(0);
    }
  }
}
