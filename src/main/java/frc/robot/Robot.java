/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**[LUCAS GRIS] TESTAR MOVIMENTAÇÃO DO CHASSIIIIIII!!! MUITAS COISAS INVERTIDAS!
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

class Robot extends TimedRobot {
/*PORTS
   */
  int kPilotstick1Port = 0;
  int kPilotstick2Port = 0;
  int kCopilotstick3Port = 1;

  int kMotorLeftPort = 9;
  int kMotorRightPort = 0;
 
  int kIntakePort = 5;

  int kPanelPort = 4;

  int kClimbPort = 3;

  /*BUTTONS  m_pilotStick1 
   */
  int kEnableSetpoint1ButtonPilot = 1;
  int kEnableSetpoint2Button = 2;
  int kEnableSetpoint3Button = 3;
  int kEnableSetpoint4Button = 4;
  int kEnableSetpoint5Button = 5;

  int kEnablePIDmoveButton = 8;

  /*BUTTONS  m_copilotStick
   */
  int kIntakeButton1 = 1;
  int KShooterPneumoButton =2;
  int kIntakeButtonreverse =10 ;
  
  int KEnablePanelPneumoButton = 5;
  int kEnablePanelRotationButton = 6 ;


  int kEnableClimbButton = 7;
  int kEnablePneumaticClimbButton = 8;
  int kEnableReverseClimbButton = 9;

  /*VELOCITY
   */
  double kIntakevelocity = 0.5 ;
  double kPanelvelocity = 0.5;


  
  /*SETPOINTS
   */
  int kPositionSetpoint1 = 0;
  int kPositionSetpoint2 = 90;
  int kPositionSetpoint3 = -90;
  int kPositionSetpoint4 = 170;
  int kPositionSetpoint5 = -170;

  
 
  
  
  

  static Timer m_timer = new Timer();
  
  Compressor m_compressor = new Compressor();
  DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(0,1);

  Spark m_chassiMotorsLeft = new Spark(kMotorLeftPort);
  Spark m_chassiMotorsRight = new Spark(kMotorRightPort);
  DifferentialDrive m_chassiDrive = new DifferentialDrive(m_chassiMotorsLeft, m_chassiMotorsRight);
  static final double kTurnP = .0045;
  static final double kTurnI = .006;
  static final double kTurnD = .0001;
  double setpoint = 0.0;
  final AHRS m_gyro = new AHRS(I2C.Port.kOnboard);
  final PIDController m_pidTurnController = new PIDController(kTurnP, kTurnI, kTurnD);

  Talon m_climbMotor = new Talon(kClimbPort);

  Talon m_panelMotor = new Talon(kPanelPort);

  Talon m_intakeMotor = new Talon (kIntakePort);

  final Joystick m_pilotStick1 = new Joystick(kPilotstick1Port);
  final Joystick m_pilotStick2 = new Joystick(kPilotstick2Port);
  final Joystick m_copilotStick = new Joystick(kCopilotstick3Port);

  double zRotation = 0.0;

  @Override
  public void robotInit() {
    m_pilotStick2.setYChannel(5);
    m_pilotStick2.setThrottleChannel(2);
    m_compressor.start();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_timer.start();
  }

  void listenSetpointButtons() {
    if (m_pilotStick1.getRawButton(kEnableSetpoint1ButtonPilot)) {
      m_pidTurnController.setSetpoint(kPositionSetpoint1);
      setpoint = kPositionSetpoint1;
    }
    if (m_pilotStick1.getRawButton(kEnableSetpoint2Button)) {
      m_pidTurnController.setSetpoint(kPositionSetpoint2);
      setpoint = kPositionSetpoint2;
    }
    if (m_pilotStick1.getRawButton(kEnableSetpoint3Button)) {
      m_pidTurnController.setSetpoint(kPositionSetpoint3);
      setpoint = kPositionSetpoint3;
    }
    if (m_pilotStick1.getRawButton(kEnableSetpoint4Button)) {
      m_pidTurnController.setSetpoint(kPositionSetpoint4);
      setpoint = kPositionSetpoint4;
    }
    if (m_pilotStick1.getRawButton(kEnableSetpoint5Button)) {
      m_pidTurnController.setSetpoint(kPositionSetpoint5);
      setpoint = kPositionSetpoint5;
    }
  }

  void listenChassiMovementButtons(){
    if (m_pilotStick1.getRawButton(kEnablePIDmoveButton)) {
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(0.0, zRotation);
    } else {
      m_chassiDrive.tankDrive(m_pilotStick1.getY(), m_pilotStick2.getY(), true);
      zRotation = 0;
    }
  }

  void listenIntakeShooterButtons(){
    if(m_copilotStick.getRawButton(kIntakeButton1)){
      m_intakeMotor.set(kIntakevelocity);
    }
    if(m_copilotStick.getRawButton(kIntakeButtonreverse)){
      m_intakeMotor.set(-kIntakevelocity);
    }
  }
   /*Shooter pneumatic. We had already declared int "kShooterPneumoButton".
    if(m_copilotStick.getRawButton(kShooterPneumoButton)){
     m_doubleSolenoid.set(value);
   }*/
  
  void listenControlPanelButton (){
    // if(m_copilotStick.getRawButton(KEnablePanelPneumoButton)){
    //   m_doubleSolenoid.set(value);
    // }      

    if(m_copilotStick.getRawButton(kEnablePanelRotationButton)){
      m_panelMotor.set(kPanelvelocity);
    }
  }

  void listenClimbButton (){
    if (m_copilotStick.getRawButton(kEnableClimbButton)) {
      m_climbMotor.set(m_copilotStick.getY());
    }
  }
  
  @Override
  public void teleopPeriodic() {
    listenSetpointButtons();
    
    listenChassiMovementButtons();
    
    listenIntakeShooterButtons();

    listenControlPanelButton();
  
    listenClimbButton();
    
    log();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    System.out.println(m_pilotStick2.getThrottle());
    System.out.println(m_pilotStick1.getThrottle());
    SmartDashboard.putNumber("Angle", m_gyro.getYaw());
    SmartDashboard.putNumber("Turning Value", zRotation);
    SmartDashboard.putNumber("SetPoint", setpoint);
  }

  void log() {
    System.out.printf("%f,%f,%f,%f,%f,%f,%f,%b\n", Robot.m_timer.get(), this.m_pidTurnController.getP(),
        this.m_pidTurnController.getI(), this.m_pidTurnController.getD(), this.m_gyro.pidGet(), this.zRotation,
        this.m_pidTurnController.getSetpoint(), this.m_pidTurnController.atSetpoint());
  }

}

