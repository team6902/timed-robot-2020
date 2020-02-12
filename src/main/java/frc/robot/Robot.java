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
import edu.wpi.first.wpilibj.SpeedController;
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
  int kMotorLeftPort = 8;
  int kMotorRightPort = 9;

  int kRedLinePort = 1;
  int kMiniCIMPort = 2;

  int kMotorArmPort = 3;

  int kClimbPort = 4;
  int kClimbPort2 = 5;

  /*
   * BUTTONS m_pilotStick1
   */
  int kNextSetPoint = 2;
  int kEnablePIDmoveButton = 1;
  int kIntakeButton = 6;

  /*
   * BUTTONS m_copilotStick
   */

  int kEnableClimbButton = 6;
  int kEnablePneumaticClimbButton = 8;
  int kEnableReverseClimbButton = 5;
  int kArmButtonUp = 1;// PID//
  int kArmButtonDown = 2;// PID//
  /*
   * int kPotentiometeroArmShooterDown = 2; //PID// int kPotentiometerArmClimb =
   * -1 ; //PID// //escolher botao
   */

  /*
   * VELOCITY
   */
  double kIntakeRedLineVelocity = 0.3;
  double kIntakeMiniCIMVelocity = 0.6;
  double kShootervelocity = 1;
  double kClimbVelocity = 0.4;
  double kClimReverseVelocity = 0.4;
  double ArmVelocityPID = 0.3;

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

  /* TIMER */
  static Timer m_timer = new Timer();

  /* PNEUMATIC */
  // Compressor m_compressor = new Compressor();
  // DoubleSolenoid m_climbdoubleSolenoid = new DoubleSolenoid(2, 3);
  /*
   * 
   * 
   * /* CHASSI MOVEMENT- SPARKS-PID-DOUBLEROTAT
   */
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

  /* PID Arm TEMOS QUE TESTAR OS VALORES */
  static final double kcP = .1;
  static final double kcI = .0;
  static final double kcD = .001;
  final PIDController m_pidArm = new PIDController(kcP, kcI, kcD);
  double setpointClimb = 0.0;// [Or barel] ClimbArm setpoint(s) para vc definir, Lucas!

  /* movimento PID infravermelho */

  private final MedianFilter m_filter = new MedianFilter(15);
  private final AnalogInput m_infrared = new AnalogInput(kInfraredPort);
  private final PIDController m_pidInfraRedController = new PIDController(kInfraredP, kInfraredI, kInfraredD);

  /* MOTORS */

  /* Intake e Shooter (1 victor) */
  VictorSP m_intakeRedLineMotor = new VictorSP(kRedLinePort);
  VictorSP m_intakeMiniCIMMotor = new VictorSP(kMiniCIMPort);
  /* Talon m_shooterMotor = new Talon (kShooterPort); */

  /* Climbing (2 talons) */
  Talon m_climbMotor1 = new Talon(kClimbPort);
  Talon m_climbMotor2 = new Talon(kClimbPort2);
  SpeedControllerGroup m_climbMotors = new SpeedControllerGroup(m_climbMotor1, m_climbMotor2);

  /* ARM (1 victor) */
  VictorSP m_MotorArm = new VictorSP(kMotorArmPort);

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
    m_pidInfraRedController.setSetpoint(distance);
    return m_pidInfraRedController.calculate(m_filter.calculate(m_infrared.getVoltage()));
  }

  /* Pneumatica */
  Compressor comp = new Compressor();
  DoubleSolenoid solenoid = new DoubleSolenoid(0, 1);
  SpeedControllerGroup climb = new SpeedControllerGroup(new Talon(4), new Talon(5));

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

    if (m_pilotStick.getRawButton(kIntakeButton)) {
      m_intakeRedLineMotor.set(kIntakeRedLineVelocity);
      m_intakeMiniCIMMotor.set(-kIntakeMiniCIMVelocity);
    } else {
      m_intakeRedLineMotor.set(0);
      m_intakeMiniCIMMotor.set(m_pilotStick.getThrottle());
    }
  }

  void listenArmMovements() {
    if (m_copilotStick.getRawButton(kArmButtonUp)) {
      m_MotorArm.set(0.3);
    } else if (m_copilotStick.getRawButton(kArmButtonDown)) {
      m_MotorArm.set(-.15);
    } else {
      m_MotorArm.set(m_copilotStick.getY() + .07);
    }
  }

  void listenClimbButton() { // [israhdahrouj] DEFINIR BOTÕES COM COPILOTO
    if (m_copilotStick.getRawButton(-1)) {
      solenoid.set(DoubleSolenoid.Value.kForward);
    }
    if (m_copilotStick.getRawButton(-1)) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    if (m_copilotStick.getRawButton(-1)) {
      climb.set(.6);
    } else if (m_copilotStick.getRawButton(-1)) {
      climb.set(-.6);
    } else {
      climb.set(0);
    }
  }

  /*
   * void listenShooterButtons() { double pidOutput =
   * m_pidArm.calculate(m_MotorArm.getDistance()); zRotation = pidOutput;
   * //if(m_copilotStick.getRawButtonpressed(kEncoderArmShooterUp)){ //
   * m_encoderArm.set(m_pidArmShooterController); //else
   * if(m_copilotStick.getRawButtonPressed(kEncoderArmShooterDown)){ //
   * m_encoderArm.set(-m_pidArmShooterController); }
   */

  @Override
  public void teleopPeriodic() {
    listenSetpointButtons();

    listenChassiMovementButtons();

    listenIntakeButtons();

    listenArmMovements();

    listenClimbButton();
    /*
     * listenShooterButtons();
     * 
     * log();
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
