package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  /*
   * PORTS
   */
  /* portas USB */
  int kPilotstickPort = 0;
  int kCopilotstickPort = 1;

  /* portas ROBORIO */
  int kMotorLeftPort = 1;
  int kMotorRightPort = 0;

  int kIntakePort = 5;


  int kPanelPort = 6;

  int kClimbPort = 2;
  int kClimbPort2 = 3;

  int kEncoderPort = 4;

  /*
   * BUTTONS m_pilotStick1
   */
  int kNextSetPoint = 2;
  int kEnablePIDmoveButton = 1;
  int kIntakeButton = 6;

  /*
   * BUTTONS m_copilotStick
   */

  int KEnablePanelUpButton = 9;

  int kEnableClimbButton = 6;
  int kEnablePneumaticClimbButton = 8;
  int kEnableReverseClimbButton = 5;
  int kEncoderArmShooterUp = 1;//PID//
  int kEncoderArmShooterDown = 2; //PID//
  int kEncoderArmClimb  = -1 ; //PID//  //escolher botao

  /*
   * VELOCITY
   */
  double kIntakevelocity = 0.5;
  double kPanelUpvelocity = 0.5;
  double kPanelRotationvelocity = 1;
  double kShootervelocity = 1;
  double kClimbVelocity = 0.4;
  double kClimReverseVelocity = 0.4;

  /*
   * constantes
   */
  private static final int kInfraredPort = 0; /* ajustar */
  static final double kHoldDistanceBottomPort = 1; /* [Fabio] ajustar */
  static final double kHoldDistanceControlPanel = 1; /* [Fabio] ajustar */
  static final double kHoldDistanceAutonomous = 1; /* ajustar */

  /* CONSTANTES PID INFRAVERMELHO */

  private static final double kInfraredP = 7.0;
  private static final double kInfraredI = 0.018;
  private static final double kInfraredD = 1.5;

  /*
   * BOOLEANS
   */
  /*
   * [Or Barel] Talvez não vamos usar motor no Shooter e sim PNEUMÁTICA(voids e
   * variaveis de shooter motor comentadas tbm)- variavel de iniciaçao do shooter
   * MOTOR (começando fechado) boolean shooterIsOpen = false;
   */

  /* variavel de iniciaçao do shooter PNEUMÁTICO (começando fechado) */
  boolean PneumoShooterIsOpen = false;
  /*
   * variavel de iniciaçao do pneumatico-panel (começando desligado) boolean
   * pneumoIsActive = false; ESSA LINHA FOI COMNTD PQ NÓS VAMOS USAR UM MOTOR AO
   * INVÉS DE PNEUMO PARA PANEL
   */
  boolean PanelIsUp = false;

  /* TIMER */
  static Timer m_timer = new Timer();

  /* PNEUMATIC */
  // Compressor m_compressor = new Compressor();
  // DoubleSolenoid m_shooterdoubleSolenoid = new DoubleSolenoid(0, 1);
  // DoubleSolenoid m_climbdoubleSolenoid = new DoubleSolenoid(2, 3);
  /*
   * DoubleSolenoid m_panelodoubleSolenoid = new DoubleSolenoid(4,5); O ROTATION
   * MOTOR DA PANEL VAI SER LEVANTADO POR OUTRO MOTOR
   */

  /* CHASSI MOVEMENT- SPARKS-PID-DOUBLEROTAT */
  Spark m_chassiMotorsLeft = new Spark(kMotorLeftPort);
  Spark m_chassiMotorsRight = new Spark(kMotorRightPort);
  DifferentialDrive m_chassiDrive = new DifferentialDrive(m_chassiMotorsLeft, m_chassiMotorsRight);
  static final double kTurnP = .1;
  static final double kTurnI = .0;
  static final double kTurnD = .001;
  double setpoint = 0.0;
  final AHRS m_gyro = new AHRS(I2C.Port.kOnboard);
  final PIDController m_pidTurnController = new PIDController(kTurnP, kTurnI, kTurnD);
  double zRotation = 0.0;

  /* PID Encoder Shooter */
  static final double kP = .1;
  static final double kI = .0;
  static final double kD = .001;
  final PIDController m_pidArmShooterController = new PIDController(kP, kI, kD);
  double setpointShooter = 0.0;//[Or barel] ShooterArm setpoint(s) para vc definir, Lucas!

  /* PID Encoder Climb */
  static final double kcP = .1;
  static final double kcI = .0;
  static final double kcD = .001;
  final PIDController m_pidArmClimbController = new PIDController(kcP, kcI, kcD);
  double setpointClimb = 0.0;//[Or barel] ClimbArm setpoint(s) para vc definir, Lucas!

  /* movimento pid infravermelho */

  private final MedianFilter m_filter = new MedianFilter(15);
  private final AnalogInput m_infrared = new AnalogInput(kInfraredPort);
  private final PIDController m_pidController = new PIDController(kInfraredP, kInfraredI, kInfraredD);


  /* MOTORS & TALONS */

  /* Intake e Shooter (1 talon) */
  Talon m_intakeMotor = new Talon(kIntakePort);
  /* Talon m_shooterMotor = new Talon (kShooterPort); */

  /* Control Panel (1 talon) */
  Talon m_panelMotor = new Talon(kPanelPort);

  /* Climbing (2 victor) */
  VictorSP m_climbMotor1 = new VictorSP(kClimbPort);
  VictorSP m_climbMotor2 = new VictorSP(kClimbPort2);
  SpeedControllerGroup m_climbMotors = new SpeedControllerGroup(m_climbMotor1, m_climbMotor2);

  /* Shooter & Climb ARM */
  Encoder m_encoderArm = new Encoder(kEncoderPort);

  /* JOYSTICK */
  final Joystick m_pilotStick = new Joystick(kPilotstickPort);
  final Joystick m_copilotStick = new Joystick(kCopilotstickPort);
  final Joystick m_pilotStick1 = new Joystick(kPilotstickPort);

  double nextSetpoint() {
    if (setpoint == 0)
      return 90.;
    if (setpoint == 90)
      return 180.;
    if (setpoint == 180)
      return -90.;
    if (setpoint == -90)
      return 0.;
    return 0.;
  }

  /* limitador de velocidade */
  public static double limit(double velocity, double limit) {
    if (velocity > limit)
      return limit;
    if (velocity < -limit)
      return -limit;
    return velocity;
  }

  public double getpidInfraredOutput(double distance) {
    m_pidController.setSetpoint(distance);
    return m_pidController.calculate(m_filter.calculate(m_infrared.getVoltage()));
  }

  @Override
  public void robotInit() {
    m_pidTurnController.enableContinuousInput(-180, 180);
    m_pilotStick.setXChannel(4);
    // m_compressor.start();
    m_pidTurnController.setTolerance(.05);
    CameraServer.getInstance().startAutomaticCapture();
    m_pilotStick.setThrottleChannel(3);
    m_pilotStick1.setThrottleChannel(2);
    m_copilotStick.setThrottleChannel(2);
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
    m_pidTurnController.setTolerance(.08);
  }

  void listenSetpointButtons() {
    if (m_pilotStick.getRawButton(8))
      m_gyro.reset();
    if (m_pilotStick.getRawButtonPressed(kNextSetPoint)) {
      setpoint = nextSetpoint();
    } else if (m_pilotStick.getPOV() != -1) {
      setpoint = m_pilotStick.getPOV();
      if (setpoint == 270)
        setpoint = -90;
    }
    System.out.println(setpoint);
    m_pidTurnController.setSetpoint(setpoint);
  }

  void listenChassiMovementButtons() {
    if (m_pilotStick.getRawButton(kEnablePIDmoveButton)) {
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(-m_pilotStick.getY(), limit(zRotation, 0.6));
    } else if (m_pilotStick.getPOV() != -1) {
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(-0.7, limit(zRotation, 0.6));
    } else {
      m_chassiDrive.arcadeDrive(-m_pilotStick.getY(), m_pilotStick.getX(), true);
      zRotation = 0;
    }
    // if (m_copilotStick.getY() != 0 || m_copilotStick.getX() != 0) {
    // m_chassiDrive.arcadeDrive(limit(m_copilotStick.getY(), 0.3),
    // limit(m_copilotStick.getX(), 0.3), true);
    // }
  }

  void listenIntakeButtons() {
    m_intakeMotor.set(m_pilotStick.getThrottle());

    if (m_pilotStick.getRawButton(kIntakeButton)) {
      m_intakeMotor.set(kIntakevelocity);
    }

    m_intakeMotor.set(-m_pilotStick1.getThrottle());
  }

  void listenShooterButtons() {
    double pidOutput = m_pidArmShooterController.calculate(m_encoderArm.getDistance());
    zRotation = pidOutput;
    //if(m_copilotStick.getRawButtonpressed(kEncoderArmShooterUp)){
     // m_encoderArm.set(m_pidArmShooterController);
    //else if(m_copilotStick.getRawButtonPressed(kEncoderArmShooterDown)){
     // m_encoderArm.set(-m_pidArmShooterController);
    }

  }
  }

  void listenControlPanelButton() {

    /*
     * VAMOS USAR MOTOR AO INVÉS DE PNEUMO-
     * if(m_copilotStick.getRawButtonPressed(KEnablePanelPneumoButton)&&
     * !pneumoIsActive ){ m_panelodoubleSolenoid.set(DoubleSolenoid.Value.kForward);
     * pneumoIsActive = true; } else
     * if(m_copilotStick.getRawButtonPressed(KEnablePanelPneumoButton) &&
     * pneumoIsActive){ m_panelodoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
     * pneumoIsActive = false; }
     */

    if (m_copilotStick.getRawButtonPressed(KEnablePanelUpButton) && !PanelIsUp) {
      m_panelMotor.set(kPanelUpvelocity);
      PanelIsUp = true;
    } else if (m_copilotStick.getRawButtonPressed(KEnablePanelUpButton) && PanelIsUp) {
      m_panelMotor.set(-kPanelUpvelocity);
      PanelIsUp = false;
    }
    // if (m_copilotStick.getRawButton(kEnablePanelRotationButton)) {
    // m_panelRotationMotor.set(kPanelRotationvelocity);
    // } else
    // m_panelRotationMotor.set(0);
  }

  void listenClimbButton() {
    /*if (m_copilotStick.getRawButton(kEnableClimbButton)) {
      m_climbMotor1.set(kClimbVelocity);
    }*/ 
    if (m_copilotStick.getRawButtonPressed(kEncoderArmClimb)){
      m_encoderArm.set(m_pidArmClimbController);
    }
    
   // double pidOutput = m_pidArmClimbController.calculate(m_encoderArm.GetDistance());
    //zRotation = pidOutput;
     if (m_copilotStick.getRawButton(kEnablePneumaticClimbButton)) {
     m_climbdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    if (m_copilotStick.getRawButton(kEnableReverseClimbButton)) {
      m_climbMotor2.set(kClimReverseVelocity);
    }
  }

  @Override
  public void teleopPeriodic() {
    listenSetpointButtons();

    listenChassiMovementButtons();

    /*
     * listenIntakeButtons();
     * 
     * listenShooterButtons();
     * 
     * listenControlPanelButton();
     * 
     * log(); listenClimbButton();
     */

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
