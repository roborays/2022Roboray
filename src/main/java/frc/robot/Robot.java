// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
  private final CANSparkMax m_leftMotor = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_rightMotor = new CANSparkMax(4, MotorType.kBrushed);
  private final CANSparkMax m_leftMotor2 = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_rightMotor2 = new CANSparkMax(3, MotorType.kBrushed);
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftMotor, m_leftMotor2);
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightMotor, m_rightMotor2);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_operatorController = new XboxController(1);

  private final WPI_TalonFX m_shooter = new WPI_TalonFX(9);
  private final WPI_TalonFX m_elevator = new WPI_TalonFX(8);

  private final WPI_TalonSRX m_intake = new WPI_TalonSRX(5);

  private final DoubleSolenoid m_hang = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 4);
  private final DoubleSolenoid m_extend = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);

  // Speed Constants
  private final double intakeSpeed = .7;
  private final double elevatorSpeed = 0.20;
  private final double shooterSpeed = -0.25;

  // Auto Time Contants
  private Timer timer;
  private double step1Time = 0; //how long the robot waits
  private double step2Time = step1Time + .2; //robot move forward and backward
  private double step3Time = step2Time + .2; //robot move forward and backward
  private double step4Time = step3Time + 3; //how long the robot shoots
  private double step5Time = step4Time + 3; //how long the robot backs up 
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(false);
    m_leftMotor.setInverted(true);
    m_rightMotor2.setInverted(false);
    m_leftMotor2.setInverted(true);

    m_extend.set(Value.kReverse);

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
          m_left.set(.4);
          m_right.set(.4);
        } else if (timer.get() <= step3Time) {
          // Run the elevator and the shooter
          m_left.set(-.4);
          m_right.set(-.4);
        }else if (timer.get() <= step4Time) {
          // Run the elevator and the shooter
          m_elevator.set(elevatorSpeed);
          m_shooter.set(shooterSpeed);
        } else if (timer.get() <= step5Time) {
          // Stop the elevator and the shooter.  Drive backwards
          m_elevator.set(0);
          m_shooter.set(0);
          m_robotDrive.tankDrive(.5, .5);
        } else {
          // Stop the drivetrain
          m_robotDrive.tankDrive(0, 0);
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
    m_robotDrive.tankDrive(-m_driverController.getLeftY(),-m_driverController.getRightY(), true);

    if (m_driverController.getAButton()){
      m_robotDrive.tankDrive(-.4, -.4);
     }

    // This uses 2 buttons to ext}end and retract a cylinder
    if (m_driverController.getLeftBumperPressed()) {
      m_hang.set(Value.kForward);
    }
    if (m_driverController.getRightBumperPressed()) {
      m_hang.set(Value.kReverse);
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
    if  (m_operatorController.getRightTriggerAxis() > 0.5){
      m_shooter.set(shooterSpeed);
    } else {
      m_shooter.set(0);
    }
    
    if (m_operatorController.getRightBumper()) {
      m_elevator.set(elevatorSpeed); 
    }
    else {
      m_elevator.set(0);
    }

    if (m_operatorController.getLeftBumper()) {
      m_intake.set(intakeSpeed);
    } else {
      m_intake.set(0);
    }

     //slow down speed of drive moto

      if (m_operatorController.getLeftTriggerAxis() > 0.5){
        m_robotDrive.tankDrive(-m_driverController.getLeftY()*0.5,-m_driverController.getRightY()*0.5, true);
    
      }
  
 /*
    if (m_operatorController.getAButton()){
      m_elevator.set(-.20);
    }

    if (m_operatorController.getBButton()){
      m_shooter.set(.35);
    }
*/
    if (m_operatorController.getAButton()){
      m_intake.set(-.7);
    }

    if (m_operatorController.getYButton()){
      m_shooter.set(.35);
      m_elevator.set(-.20);
    }
  
  }
}
