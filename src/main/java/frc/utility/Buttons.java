package frc.utility;  // Change from frc.robot to frc.utility
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Buttons {
    // Controllers
    private final CommandXboxController driverXbox;
    private final CommandXboxController operatorXbox;
    
    // Driver Button triggers
    private final Trigger driverA;
    private final Trigger driverB;
    private final Trigger driverX;
    private final Trigger driverY;
    private final Trigger driverLeftBumper;
    private final Trigger driverRightBumper;
    private final Trigger driverLeftTrigger;
    private final Trigger driverRightTrigger;
    private final Trigger driverBack;
    private final Trigger driverStart;
    private final Trigger driverPovUp;
    private final Trigger driverPovDown;
    private final Trigger driverPovRight;
    private final Trigger driverPovLeft;
    
    // Operator Button triggers
    private final Trigger operatorA;
    private final Trigger operatorB;
    private final Trigger operatorX;
    private final Trigger operatorY;
    
    public Buttons() {
        // Initialize controllers
        driverXbox = new CommandXboxController(0);
        operatorXbox = new CommandXboxController(1);
        
        // Initialize driver buttons
        driverA = driverXbox.a();
        driverB = driverXbox.b();
        driverX = driverXbox.x();
        driverY = driverXbox.y();
        driverLeftBumper = driverXbox.leftBumper();
        driverRightBumper = driverXbox.rightBumper();
        driverLeftTrigger = driverXbox.leftTrigger();
        driverRightTrigger = driverXbox.rightTrigger();
        driverBack = driverXbox.back();
        driverStart = driverXbox.start();
        driverPovUp = driverXbox.povUp();
        driverPovDown = driverXbox.povDown();
        driverPovRight = driverXbox.povRight();
        driverPovLeft = driverXbox.povLeft();
        
        // Initialize operator buttons
        operatorA = operatorXbox.a();
        operatorB = operatorXbox.b();
        operatorX = operatorXbox.x();
        operatorY = operatorXbox.y();
    }
    
    // Getters for controllers
    public CommandXboxController getDriverXbox() {
        return driverXbox;
    }
    
    public CommandXboxController getOperatorXbox() {
        return operatorXbox;
    }
    
    // Getters for driver buttons
    public Trigger getDriverA() { return driverA; }
    public Trigger getDriverB() { return driverB; }
    public Trigger getDriverX() { return driverX; }
    public Trigger getDriverY() { return driverY; }
    public Trigger getDriverLeftBumper() { return driverLeftBumper; }
    public Trigger getDriverRightBumper() { return driverRightBumper; }
    public Trigger getDriverLeftTrigger() { return driverLeftTrigger; }
    public Trigger getDriverRightTrigger() { return driverRightTrigger; }
    public Trigger getDriverBack() { return driverBack; }
    public Trigger getDriverStart() { return driverStart; }
    public Trigger getDriverPovUp() { return driverPovUp; }
    public Trigger getDriverPovDown() { return driverPovDown; }
    public Trigger getDriverPovRight() { return driverPovRight; }
    public Trigger getDriverPovLeft() { return driverPovLeft; }
    public Trigger getOperatorPovRight() { return operatorXbox.povRight(); }
    public Trigger getOperatorPovLeft() { return operatorXbox.povLeft(); }
    
    // Getters for operator buttons
    public Trigger getOperatorA() { return operatorA; }
    public Trigger getOperatorB() { return operatorB; }
    public Trigger getOperatorX() { return operatorX; }
    public Trigger getOperatorY() { return operatorY; }
    
    // Raw axis access
    public double getDriverLeftX() { return driverXbox.getLeftX(); }
    public double getDriverLeftY() { return driverXbox.getLeftY(); }
    public double getDriverRightX() { return driverXbox.getRightX(); }
    public double getDriverRightY() { return driverXbox.getRightY(); }
}