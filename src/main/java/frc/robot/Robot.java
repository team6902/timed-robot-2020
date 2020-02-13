package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

  NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  /* PORTS */

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
  int kServoPort = 7;

  /* BUTTONS m_pilotStick */
  int kNextSetPoint = 2;
  int kEnablePIDmoveButton = 1;
  int kIntakeButton = 6;
  int kIntakeReverseButton = 5;

  /* BUTTONS m_copilotStick */
  int kArmButtonUp = 1;// PID//
  int kArmButtonDown = 2;// PID//
  int kButtonMotorClimbActivated = 6;
  int kButtonMotorClimbReverse = 5;

  /* VELOCITY */
  double kIntakeRedLineVelocity = 0.3;
  double kIntakeMiniCIMVelocity = 0.6;
  double kClimbVelocity = 0.6;
  double kArmUpVelocity = 0.3;
  double kArmDownVelocity = -0.15;
  double kArmVelocity = .07;

  /* CONSTANTES */
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

  /* SERVO */
  Servo servo = new Servo(kServoPort);

  /* BOOLEAN SERVO */
  boolean servoB = false;

  /* PNEUMATIC */
  Compressor m_compressor = new Compressor();
  DoubleSolenoid solenoid = new DoubleSolenoid(0, 1);

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

  /* MOVIMENTO PID INFRAVERMELHO */
  private final MedianFilter m_filter = new MedianFilter(15);
  private final AnalogInput m_infrared = new AnalogInput(kInfraredPort);
  private final PIDController m_pidInfraRedController = new PIDController(kInfraredP, kInfraredI, kInfraredD);

  /* MOTORS */

  /* INTAKE (1 victor) */
  VictorSP m_intakeRedLineMotor = new VictorSP(kRedLinePort);
  VictorSP m_intakeMiniCIMMotor = new VictorSP(kMiniCIMPort);

  /* CLIMB (2 talons) */
  Talon m_climbMotor1 = new Talon(kClimbPort);
  Talon m_climbMotor2 = new Talon(kClimbPort2);
  SpeedControllerGroup m_climbMotors = new SpeedControllerGroup(m_climbMotor1, m_climbMotor2);

  /* ARM (1 victor) */
  VictorSP m_MotorArm = new VictorSP(kMotorArmPort);

  /* JOYSTICK */
  final Joystick m_pilotStick = new Joystick(kPilotstickPort);
  final Joystick m_copilotStick = new Joystick(kCopilotstickPort);
  final Joystick m_pilotStick1 = new Joystick(kPilotstickPort);
  final Joystick m_copilotStick1 = new Joystick(kCopilotstickPort);
  final Joystick m_copilotStick2 = new Joystick(kCopilotstickPort);
  final Joystick m_copilotStick3 = new Joystick(kCopilotstickPort);

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

  static final double kPixyP = .01;
  static final double kPixyI = .0;
  static final double kPixyD = .001;
  final PIDController m_pidPixyController = 
      new PIDController(kPixyP, kPixyI, kPixyD);

  
  static final double kPixyAreaP = .1;
  static final double kPixyAreaI = .0;
  static final double kPixyAreaD = .001;
  final PIDController m_pidPixyControllerArea = 
      new PIDController(kPixyP, kPixyI, kPixyD);
  

  /* LIMITADOR DE VELOCIDADE */
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
    m_copilotStick1.setThrottleChannel(3);
    m_copilotStick2.setThrottleChannel(1);
    m_copilotStick3.setThrottleChannel(4);
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
    m_pidTurnController.setSetpoint(setpoint);
  }

  void grabPowerCell() {
    NetworkTable table = tableInstance.getTable("SmartDashboard");
    double x = table.getEntry("x").getDouble(0);
    double y = table.getEntry("y").getDouble(0);
    double width = table.getEntry("width").getDouble(0);
    double height = table.getEntry("height").getDouble(0);
    System.out.println(x + " " + y);
    // double centerX = x + width/2.0;
    double centerX = x - 100;
    double area = width * height;

    double pidOutputX = m_pidPixyController.calculate(centerX);
    double pidOutputArea = m_pidPixyControllerArea.calculate(area);
    m_chassiDrive.arcadeDrive(limit(pidOutputArea, 0.5), 
        -limit(pidOutputX, 0.7));
    // System.out.println(centerX + " " + pidOutputX);
    System.out.println(area + " " + pidOutputArea);

  }

  void listenChassiMovementButtons() {
    if (m_pilotStick.getRawButton(kEnablePIDmoveButton)) {
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(m_pilotStick.getY(), limit(zRotation, 0.6));
    } else if (m_pilotStick.getPOV() != -1) {
      double pidOutput = m_pidTurnController.calculate(m_gyro.pidGet());
      zRotation = pidOutput;
      m_chassiDrive.arcadeDrive(0.7, limit(zRotation, 0.6));
    } else {
      m_chassiDrive.arcadeDrive(m_pilotStick.getY(), m_pilotStick.getX(), true);
      zRotation = 0;
    }
    if (m_copilotStick.getThrottle()!= 0 || m_copilotStick1.getThrottle() != 0) {
      m_chassiDrive.arcadeDrive((limit(m_copilotStick2.getThrottle(), 0.7)), limit(m_copilotStick3.getThrottle(), 0.7), true);
    }

    // ToDo: trocar botão?
    // ToDo: aumentar ganho do PID e adicionar código de intake
    if (m_pilotStick.getRawButton(4)) { 
      grabPowerCell();  
    }
  }

  void listenServo() {
    if (m_pilotStick.getRawButtonPressed(3)) {
      servoB = !servoB;
    }
    if (servoB) {
      servo.setAngle(0);
    } else if (!servoB) {
      servo.setAngle(180);
    }
  }

  void listenIntakeButtons() {
    if (m_pilotStick.getRawButton(kIntakeButton)) {
      m_intakeMiniCIMMotor.set(kIntakeMiniCIMVelocity);
    } 

    else if (m_pilotStick.getRawButton(kIntakeReverseButton)) {
      m_intakeMiniCIMMotor.set(-kIntakeMiniCIMVelocity);
    } 
    else m_intakeMiniCIMMotor.set(0);

    if (m_pilotStick.getThrottle() > 0) {
      m_intakeRedLineMotor.set(-m_pilotStick.getThrottle());
    } else {
      m_intakeRedLineMotor.set(limit(m_pilotStick1.getThrottle(), 0.8));
    }
  }

  void listenArmMovements() {
    if (m_copilotStick.getRawButton(kArmButtonUp)) {
      m_MotorArm.set(kArmUpVelocity);
    } else if (m_copilotStick.getRawButton(kArmButtonDown)) {
      m_MotorArm.set(kArmDownVelocity);
    } else {
      m_MotorArm.set(kArmVelocity);
    }
  }

   void listenClimbButton() {
    if (m_copilotStick.getRawButton(kButtonMotorClimbActivated)) {
      m_climbMotors.set(-kClimbVelocity);
    } else if (m_copilotStick.getRawButton(kButtonMotorClimbReverse)) {
      m_climbMotors.set(kClimbVelocity);
    } else {
      m_climbMotors.set(0);
    }
    if (m_copilotStick1.getThrottle()>0) {
      solenoid.set(DoubleSolenoid.Value.kForward);
    }
    if (m_copilotStick.getThrottle()>0) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }

  }

  @Override
  public void teleopPeriodic() {
    listenSetpointButtons();

    listenChassiMovementButtons();

    listenIntakeButtons();

    listenArmMovements();
    
    listenClimbButton();

    listenServo();

    // log();

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
