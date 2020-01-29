package frc.robot;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid; 
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/*[Or Barel] Talvez não vamos usar motor no Shooter e sim PNEUMÁTICA(voids e variaveis de shooter motor comentadas tbm)
/**[LUCAS GRIS] TESTAR MOVIMENTAÇÃO DO CHASSIIIIIII!!! MUITAS COISAS INVERTIDAS!
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

public class Robot extends TimedRobot {
/*PORTS
   */  
  /* portas USB */
  int kPilotstickPort = 0;
  int kCopilotstickPort = 1;

  /* portas ROBORIO */
  int kMotorLeftPort = 9;
  int kMotorRightPort = 0;
 
  int kIntakePort = 5;
  /* int kShooterMotorPort= 6; */

  int kPanelPort = 4;

  int kClimbPort = 3;

  /*BUTTONS  m_pilotStick1 
   */
  int kEnableSetpoint1ButtonPilot = 1;
  int kEnableSetpoint2Button = 2;
  int kEnableSetpoint3Button = 3;
  int kEnableSetpoint4Button = 4;
  int kEnableSetpoint5Button = 5;
  int kIntakeButton = 1;
  int kIntakeButtonreverse =10 ;
  int kEnablePIDmoveButton = 8;

  /*BUTTONS  m_copilotStick
   */

  /* int KShooterMotorButton =2; */
  int KShooterPneumoButton = 2; 
 
  
 //int KEnablePanelPneumoButton = 5;
  //int kEnablePanelRotationButton = 6 ;


  int kEnableClimbButton = 6;
  int kEnablePneumaticClimbButton = 8;
  int kEnableReverseClimbButton = 5;

  /*VELOCITY
   */
  double kIntakevelocity = 0.5 ;
  double kPanelvelocity = 1;
  double kShootervelocity = 1;
  double kClimReversevelocity = 0.4;


  
  /*SETPOINTS
   */
  int kPositionSetpoint1 = 0;
  int kPositionSetpoint2 = 90;
  int kPositionSetpoint3 = -90;
  int kPositionSetpoint4 = 170;
  int kPositionSetpoint5 = -170;

  /* constantes 
  */
  private static final int kInfraredPort = 0; /* ajustar */
    static final double kHoldDistanceBottomPort = 1; /* [Fabio] ajustar */
    static final double kHoldDistanceControlPanel = 1; /* [Fabio] ajustar */
    static final double kHoldDistanceAutonomous = 1; /* ajustar */

/* CONSTANTES PID INFRAVERMELHO */



    private static final double kInfraredP = 7.0;
    private static final double kInfraredI = 0.018;
    private static final double kInfraredD = 1.5;
  
    
  
  /* BOOLEANS 
  */
  /*[Or Barel] Talvez não vamos usar motor no Shooter e sim PNEUMÁTICA(voids e variaveis de shooter motor comentadas tbm)-
  variavel de iniciaçao do shooter MOTOR (começando fechado) 
  boolean shooterIsOpen = false; */

  /* variavel de iniciaçao do shooter PNEUMÁTICO (começando fechado) */
  boolean PneumoShooterIsOpen = false;
  /* variavel de iniciaçao do pneumatico-panel (começando desligado) */
  boolean pneumoIsActive = false;

  
  /* TIMER */
  static Timer m_timer = new Timer();

  
  
  /* PNEUMATIC  */
  Compressor m_compressor = new Compressor();
  DoubleSolenoid m_shooterdoubleSolenoid = new DoubleSolenoid (0,1);
  DoubleSolenoid m_panelodoubleSolenoid = new DoubleSolenoid(2,3);
  DoubleSolenoid m_climbdoubleSolenoid = new DoubleSolenoid(4,5);
  

  /* CHASSI MOVEMENT- SPARKS-PID-DOUBLEROTAT */
  Spark m_chassiMotorsLeft = new Spark(kMotorLeftPort);
  Spark m_chassiMotorsRight = new Spark(kMotorRightPort);
  DifferentialDrive m_chassiDrive = new DifferentialDrive(m_chassiMotorsLeft, m_chassiMotorsRight);
  static final double kTurnP = .0045;
  static final double kTurnI = .006;
  static final double kTurnD = .0001;
  double setpoint = 0.0;
  final AHRS m_gyro = new AHRS(I2C.Port.kOnboard);
  final PIDController m_pidTurnController = new PIDController(kTurnP, kTurnI, kTurnD);
  double zRotation = 0.0;

  /* movimento pid infravermelho */

  private final MedianFilter m_filter = new MedianFilter(15);
  private final AnalogInput m_infrared = new AnalogInput(kInfraredPort);
  private final PIDController m_pidController = new PIDController(kInfraredP, kInfraredI, kInfraredD);


  /*MOTORS & TALONS  */

  /* Intake e Shooter  */
  Talon m_intakeMotor = new Talon (kIntakePort);

  /* Control Panel  */
  Talon m_panelMotor = new Talon(kPanelPort);
     /* Talon m_shooterMotor = new Talon (kShooterPort); */

  /* Climbing  */
  Talon m_climbMotor = new Talon(kClimbPort);


 /* JOYSTICK */
  final Joystick m_pilotStick = new Joystick(kPilotstickPort);
  final Joystick m_copilotStick = new Joystick(kCopilotstickPort);
  final Joystick m_pilotStick1 = new Joystick(kPilotstickPort);

 /* limitador de velocidade */
  public static double limit(double velocity) {
    if (velocity > .4) return .4;
    if (velocity < -.4) return -.4;
    return velocity;
  }

  public double getpidInfraredOutput (double distance) {
    m_pidController.setSetpoint(distance);
    return m_pidController.calculate(m_filter.calculate(m_infrared.getVoltage()));
  }



  @Override
  public void robotInit() {
    m_pilotStick.setXChannel(4);
    m_compressor.start();
    CameraServer.getInstance().startAutomaticCapture();
    m_pilotStick1.setThrottleChannel(2);
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
    if (m_pilotStick.getRawButton(kEnableSetpoint1ButtonPilot)) {
      m_pidTurnController.setSetpoint(kPositionSetpoint1);
      setpoint = kPositionSetpoint1;
    }
    if (m_pilotStick.getRawButton(kEnableSetpoint2Button)) {
      m_pidTurnController.setSetpoint(kPositionSetpoint2);
      setpoint = kPositionSetpoint2;
    }
    if (m_pilotStick.getRawButton(kEnableSetpoint3Button)) {
      m_pidTurnController.setSetpoint(kPositionSetpoint3);
      setpoint = kPositionSetpoint3;
    }
    if (m_pilotStick.getRawButton(kEnableSetpoint4Button)) {
      m_pidTurnController.setSetpoint(kPositionSetpoint4);
      setpoint = kPositionSetpoint4;
    }
    if (m_pilotStick.getRawButton(kEnableSetpoint5Button)) {
      m_pidTurnController.setSetpoint(kPositionSetpoint5);
      setpoint = kPositionSetpoint5;
    }

    
  }

  void listenChassiMovementButtons(){
    if (m_pilotStick.getRawButton(kEnablePIDmoveButton)) {
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(0.0, zRotation);
    } else {
      m_chassiDrive.arcadeDrive(m_pilotStick.getY(), m_pilotStick.getX(), true);
      zRotation = 0;
    }
  }

  void listenIntakeShooterButtons(){
    m_intakeMotor.set(m_pilotStick.getThrottle());

   if(m_pilotStick.getRawButton(kIntakeButton)){
      m_intakeMotor.set(kIntakevelocity);
    }

    m_intakeMotor.set(-m_pilotStick1.getThrottle());
   
   
    /*[Or Barel] Talvez não vamos usar motor no Shooter e sim PNEUMÁTICA(voids e variaveis de shooter motor comentadas tbm)
    /*if (m_copilotStick.getRawButton(KShooterButton) && !shooterIsopen) {
      m_shooterMotor.set(kShootervelocity);
      shooterIsOpen= true;
    }              
    else if(m_copilotStick.getRawButton(KShooterButton) && shooterIsOpen){
      m_shooterMotor.set(-kShootervelocity);
      shooterIsopen = false;
    }*/                    
    
    
    /* pneumatico do shooter */
    if(m_copilotStick.getRawButtonPressed(KShooterPneumoButton)&& !PneumoShooterIsOpen){
     m_shooterdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
     PneumoShooterIsOpen = true;
    } 
    else if (m_copilotStick.getRawButtonPressed(KShooterPneumoButton)&& PneumoShooterIsOpen){
      m_shooterdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      PneumoShooterIsOpen = false;
    }
    if(m_copilotStick.getRawButton(kIntakeButtonreverse)){
      m_intakeMotor.set(-kIntakevelocity);
    }
  }
  
  void listenControlPanelButton (){
     
    if(m_copilotStick.getRawButtonPressed(KEnablePanelPneumoButton)&& !pneumoIsActive ){
      m_panelodoubleSolenoid.set(DoubleSolenoid.Value.kForward);
      pneumoIsActive = true;
     }      
     else if(m_copilotStick.getRawButtonPressed(KEnablePanelPneumoButton) && pneumoIsActive){
       m_panelodoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
       pneumoIsActive = false;
     }

    if(m_copilotStick.getRawButton(kEnablePanelRotationButton)){
      m_panelMotor.set(kPanelvelocity);
    }
  }

  void listenClimbButton (){
    if (m_copilotStick.getRawButton(kEnableClimbButton)) {
      m_climbMotor.set(m_copilotStick.getY());
    }
    if(m_copilotStick.getRawButton(kEnablePneumaticClimbButton)){
      m_climbdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
     }
    if(m_copilotStick.getRawButton(kEnableReverseClimbButton)){
      m_climbMotor.set(kClimReversevelocity);
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
