from vex import *

# Brain should be defined by default
brain=Brain()

# -------------- Robot Configuration ---------------
rightMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False) # Right
leftMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)   # Left
# Set the leftMotor reverse property to True so that when driving forward
# turns in the same direction as the right motor.
driveTrain = DriveTrain(leftMotor, rightMotor)                 # Start the motors simultaneuously
liftMotor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)  # Liftarm
inertial_1 = Inertial(Ports.PORT5)                             # Inertial sensor
liftArmRotation = Rotation(Ports.PORT6, False)                 # Liftarm rotation sensor
bumpSwitch = Bumper(brain.three_wire_port.a)                   # Bumper switch
# --------------------------------------------------------------------------------------------

# ----------------------------------------------- Helper Functions --------------------------------------
def bump():
    """
    Hold the program's execution until the button is pressed
    """

    # Waiting for the bump switch to be pressed
    while(bumpSwitch.pressing() == False):
        wait(10, MSEC) # Debounce the button

        brain.screen.set_cursor(1,1)  # Moving the cursor to position (1,1)
        brain.screen.print("Press the button to start the program")
        pass
    brain.screen.clear_line(1)         # Deleting the text on line 1
    brain.screen.set_cursor(1,1)       # Moving the cursor to position (1,1)
    brain.screen.print("Program executed")
    wait(1, SECONDS)


def inertialCalibration():
    """
    1. Calibrate the inertial sensor
    2. A wait time of 2 seconds is required to calibrate the sensor
    3. The functions is called at the start of the program execution
    """

    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("Calibrating the inertial sensor")
    brain.screen.set_cursor(2,1)
    brain.screen.print("Don't move the robot")
    inertial_1.calibrate()       # Calibrate the inertial sensor

    wait(2, SECONDS)             # Time required to calibrate the sensor

    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("Calibration is complete")

def testInertial():
    """
    1. Test the inertial sensor by having it display heading and rotation data
    2. Press the  button to end the test
    """

    brain.screen.clear_screen()
    while(bumpSwitch.pressing() == False):
        wait(10, MSEC) # Debounce the button
        brain.screen.set_cursor(5,1)
        brain.screen.print("Heading: " + str(inertial_1.heading()))
        brain.screen.set_cursor(6,1)
        brain.screen.print("Rotation: " + str(inertial_1.rotation()))
        brain.screen.set_cursor(8,1)
        brain.screen.print("Press the button to end the test")
    
    brain.screen.clear_screen()
    brain.screen.set_cursor(8,1)
    brain.screen.print("Inertial test complete")


def driveStraightData(e):
    """
    1. Report position, rotation and error
    2. Paramater:   3 = error value (setpoint)
    """

    brain.screen.set_cursor(1,1)
    brain.screen.print("Position: " + str(leftMotor.position()))    # Returning position

    brain.screen.set_cursor(2,1)
    brain.screen.print("Rotation: " + str(inertial_1.rotation()))   # Returning rotation

    brain.screen.set_cursor(3,1)
    brain.screen.print("Error: " + str(e))                          # Return the current error

def stopMotors():
    """
    Stop both motors at the same time
    """

    driveTrain.stop()
    wait(0.5, SECONDS)  # Allow the robot to stabalize


def driveStraight(distance, setpoint, motorVelocity):
    """
    1. distance = distance in inches
    2. setpoint = 0-deg. for driving straight
    3. motoroVelocity = nominal velocity (+) +> Forward, (-) -> Reverse
    
    """

    inertial_1.reset_rotation()  # Reset the rotation value before moving
     # Set stopping mode for left and right motors
    leftMotor.set_stopping(COAST)
    rightMotor.set_stopping(COAST)

    kP = 0.43  # Proportional constant for driving straight
               # Used to calculate the velocity correction term
               # If too small the correction will occur too slowly
               # If too large the system will over correct
               # Determine the best by iteratively testing

    # Calculte the distance in terms of encoder count
    wheelDiameter = 4       # Units = inches
    wheelCircumference = wheelDiameter * math.pi     # Unites = inches

    # ditance(ticks) = (distance(inches)) / Wheel Circumference() * 360
    distance = (distance/wheelCircumference) * 360 # Units = ticks


    leftMotor.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)

    # Drive straight forward if motor velocity > 0
    if(motorVelocity > 0):
        # use a while loop track the distance traveled
        while(leftMotor.position() < distance):
            error = (setpoint - inertial_1.rotation())   # Rotation error
            correction = kP * error        # Motor velocity correction term

            # Correct the motor velocites to maintain course
            # If error > 0 => drive left
            # IF error < 0 => drive right

            leftMotor.set_velocity((motorVelocity + correction), PERCENT)
            rightMotor.set_velocity((motorVelocity - correction), PERCENT)

            # spin the motors
            driveTrain.drive(FORWARD)

            driveStraightData(error)      # Display currentposition, rotation & 

        stopMotors()     # Stop both motors when the desired distance is traveled
                         # You wil need to account for momentum

#
    else:
        distance *= -1   # Distance count must be negative for driving r
        # use a while loop track the distance traveled
        while(leftMotor.position() > distance):
            error = (setpoint - inertial_1.rotation())   # Rotation error
            correction = kP * error        # Motor velocity correction term

            # Correct the motor velocites to maintain course
            # If error > 0 => drive left
            # IF error < 0 => drive right

            leftMotor.set_velocity((motorVelocity + correction), PERCENT)
            rightMotor.set_velocity((motorVelocity - correction), PERCENT)

            # spin the motors
            driveTrain.drive(FORWARD)

            driveStraightData(error)      # Display currentposition, rotation & 

        stopMotors()     # Stop both motors when the desired distance is traveled
                         # You wil need to account for momentum


def turnData(turnError, derivative):
    """
    1. Print the current heading, turning error and turning derivative
    """

    brain.screen.set_cursor(1,1)
    brain.screen.print("Heading: " + str(inertial_1.heading()))    # Returning heading

    brain.screen.set_cursor(2,1)
    brain.screen.print("Error: " + str(turnError))               # Returning turnError

    brain.screen.set_cursor(3,1)
    brain.screen.print("Derivative: " + str(derivative))           # Return the derivative

def pointTurn(setPoint):
    """
    1. Perform a point turn using the inertial sensor's heading
    2. Propotional and derivative control to maintain accuracy
    """


    brain.screen.clear_screen() # Clear the screen in preparation for the turn data

     # Set stopping mode for left and right motors
    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)

    # Calculate the differernce between the setpoint and current heading
    difference = setPoint - inertial_1.heading()

    # Want to turn the smallest amount to reach the setpoint (i.e not the reflex angle)
    if (setPoint > inertial_1.heading()):         # Setpoint is greater than current heading
        if(abs(difference) <= 180):              
            clockwise = True                      # Turn CW
        else:
            clockwise = False                     # Turn CCW
    else:
        if(abs(difference <= 180)):
            clockwise = True                     # Turn CCW
        else:
            clockwise = False                      # Turn CW

    # Define P & D constant by testing the robot using CW and CCW turns
    if(clockwise):
        # Set PD constants for turning CW
        kP = 0.08
        kD = 0.8
    else:      # P & D values if turning CCW
        kP = 0.1
        kD = 0.6

    # Define the max turning velocity and previous variables
    maximumVelocity = 60        # Units:     %
    previousError = 0.0         # Error from previous run of the while loop

    while (True):

        # Propotional Term
        turnError = setPoint - inertial_1.heading()           # Calculate the error

        # Derivative
        derivative = turnError - previousError                # Current error - previous error

        # Stop motors and exit the while() loop if the error and derivative terms are sufficiently
        # small to ensure that the setpoint was reached with minimal oscillation.
        # If you only check the error term, momentum will cause the robot to overshoot the target
        # Including the derivative will ensure the motors stop after oscillaion is eliminated
        if(abs(turnError) < 1 and (abs(derivative) < 0.02)):
            stopMotors()         # Stop the motors
            break                # Exit the while() loop

        
        # Calculate the correction termf for the motor velocities
        turnCorrection = kP * turnError + kP * derivative            # P & D control

        # Limit the PD output to remain beween -1 and 1
        # This will keep the motor velocity from exceeding its maximum set value
        if(abs(turnCorrection) > 1):
            turnCorrection = 1
        
        turnVelocity = maximumVelocity * turnCorrection

        # Set motor melocities based on the direction of turn and the PD output
        if (clockwise):       # Turn CW
            leftMotor.set_velocity(turnVelocity)        # Turn the left motor forward
            rightMotor.set_velocity(-1 * turnVelocity)  # Turn the right motor in reverse

        else: 
            leftMotor.set_velocity(-1 * turnVelocity)     # Turn the left motor in reverse
            rightMotor.set_velocity(turnVelocity)         # Turn the right motor forward

        # Spin the motors
        leftMotor.spin(FORWARD)
        rightMotor.spin(FORWARD)

        turnData(turnError, derivative)     # Print current heading value

        # Update previousError
        previousError = turnError    # Set previous error = current error

        wait(20, MSEC)


def liftArm(motorVelocity, liftAngle):

    # Configure the lift motor to hold its position
    liftMotor.set_stopping(HOLD)

    liftMotor.set_velocity(motorVelocity, PERCENT)   # Set lift motor velocity

    gearRatio = 5  # 60 tooth to 12 tooh
    motorAngularDisplacement =liftAngle * gearRatio      # Calculate the motor axles angular displacement

    # Spin the motor forward for the given angular displacement
    liftMotor.spin_for(FORWARD, motorAngularDisplacement, DEGREES)
    wait (0.5, SECONDS)


 
# ---------------------------------------- Define main() -----------------------------------
def main():    
    """
    This is the function that will be executed by the brain
    """

    bump()                 # Call bump() to begin the program

    inertialCalibration()  # Calibrate the inertial sensor at the start of the program
    #testInertial()         # Test the inertial heading and rotation values

    driveStraight(101.5,0,50)
    liftArm(20, 35)
    wait(0.5,SECONDS)
    driveStraight(3, 0, -50)
    pointTurn(90)
    wait(0.5, SECONDS)
    driveStraight(70,0,50)
    pointTurn(50)
    driveStraight(13,0,50)
    liftArm(20, -35)
# ----------------------------------------------------------------------------------------

main()
