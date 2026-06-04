"""
Name:       main.py
Coded by:   Samvarth Raj, Evangelos Parameritis
Purpose:    Use GitHub and Inertial Sensor
Date:       5/8/2026
"""

from vex import *

# Brain should be defined by default
brain=Brain()

# -------------- Robot Configuration ---------------
rightMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False) # Right
leftMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)   # Left
# Set the leftMotor reverse property to True so that when driving forward
# turns in the same direction as the right motor.

drivetrain = DriveTrain(leftMotor,rightMotor)


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
    brain.screen.clear_screen
    brain.screen.set_cursor(1,1)
    brain.screen.print("Calibrating the intertial sensor")
    brain.screen.set_cursor(2,1)
    brain.screen.print("Don't move the robot!")
    inertial_1.calibrate()

    wait(2, SECONDS)

    brain.screen.clear_screen
    brain.screen.set_cursor(1,1)
    brain.screen.print("Calibration is complete")

def testInertial():
    """
    1. Test the interial sensor by having it display heading and rotation
    2. Press the button to end the test
    """
    while (bumpSwitch.pressing() == False):
        wait(10, MSEC)
        brain.screen.set_cursor(5, 1)
        brain.screen.print("Heading: " + str(inertial_1.heading()))
        brain.screen.set_cursor(6, 1)
        brain.screen.print("Roatation: " + str(inertial_1.rotation()))
        brain.screen.set_cursor(8,1)
        brain.screen.print("Press the button to end the test.")

    brain.screen.clear_screen()
    brain.screen.set_cursor(8,1)
    brain.screen.print("Inertial test complete")


def driveStraightData(e):
    """
    1. Report position, rotation and error
    2. Parameter: 3 = error value (setpoint - rotation)
    """

    brain.screen.set_cursor(1,1)
    brain.screen.print("Position: " + str(leftMotor.position())) #Return the current position

    brain.screen.set_cursor(2,1)
    brain.screen.print("Rotation: " + str(inertial_1.rotation())) #Return the current rotation

    brain.screen.set_cursor(3,1)
    brain.screen.print("Error: " + str(e)) #Return the current error


def stopMotors():
    """
    Stop both motors at the same time.
    """
    drivetrain.stop()
    wait(0.5,SECONDS) # Allow the robot to stabilize


def driveStraight(distance, setpoint, motorVelocity):
    """
    1. distance = distance in inches
    2. setpoint = 0 degrees for driving straight
    3. motorVelocity = nominal velocity (+) => Forward, (-) => Reverse
    """
    inertial_1.reset_rotation() #Reset the rotation value before moving

    leftMotor.set_stopping(COAST)
    rightMotor.set_stopping(COAST)
    
    kP = 0.35   #Constant of proportionality
                #Used to calculate velocity correction term
                #If too small, correction occurs too slow
                #If too large, system will overcorrect
                #Tune the values via testing
    
    #Calculate the distance in terms of encoder count
    wheelDiameter = 4                               # Units = inches
    wheelCircumference = wheelDiameter * math.pi    # Units = inches

    # distance(ticks) = (distance(inches) / Wheel Circumference * 360)

    distance = (distance/wheelCircumference) * 360

    #reset the motor encoders
    leftMotor.set_position(0,DEGREES)
    rightMotor.set_position(0, DEGREES)
    
    #Drive straight forward if motor velocity > 0

    if (motorVelocity > 0):
        # Use a while loop track the distance traveled
        while (leftMotor.position() < distance):
            error = (setpoint - inertial_1.rotation()) # Rotation error
            correction = kP * error

            #Correct motor velocitires to maintain course
            # If error > 0, it's drifting left
            # If error < 0, it's drifting right

            leftMotor.set_velocity((motorVelocity + correction), PERCENT)
            rightMotor.set_velocity((motorVelocity - correction), PERCENT)

            #Spin the motors
            
            drivetrain.drive(FORWARD)
            driveStraightData(error) #display current position, rotation, and error

        stopMotors()    #Stop both motors when distance is reached
                        #You will need to account for momentum
    
    # Distance must be negative to drive backwards
    else:
        distance *= -1
        # Use a while loop track the distance traveled
        while (leftMotor.position() > distance):
            error = (setpoint - inertial_1.rotation()) # Rotation error
            correction = kP * error

            #Correct motor velocitires to maintain course
            # If error > 0, it's drifting left
            # If error < 0, it's drifting right

            leftMotor.set_velocity((motorVelocity + correction), PERCENT)
            rightMotor.set_velocity((motorVelocity - correction), PERCENT)

            #Spin the motors
            drivetrain.drive(FORWARD)

            driveStraightData(error) #display current position, rotation, and error

        stopMotors()    #Stop both motors when distance is reached
                        #You will need to account for momentum    
    
def turnData(turnError, derivative):
    """
    1.  Print the current heading, turning error and turning derivative values
    """

    brain.screen.set_cursor(1,1)
    brain.screen.print("Heading: " + str(inertial_1.heading())) #Return the current position

    brain.screen.set_cursor(2,1)
    brain.screen.print("Error: " + str(turnError)) #  Current turtn error

    brain.screen.set_cursor(3,1)
    brain.screen.print("Derivative: " + str(derivative)) #  Current turn derivative

def pointTurn(setPoint):
    """
    1.  Perform a point turn using the inertial sensor's heading
    2.  Proportional and derivative control to maintain accuracy
    """

    brain.screen.clear_screen() # Clear the screen in preparation for the training data

    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)

    # Calculate the difference between the setpoint and current heading
    difference = setPoint - inertial_1.heading()

    # Want to turn the smallest amount to reach the setpoint (i.e not the reflex angle)
    if (setPoint > inertial_1.heading()): # Setpoint is greater than current heading
        if (abs(difference) <= 180): # Turn CW
            clockwise = True
        else:
            clockwise = False       # Turn CCW
    else:                            # Setpoint is less than the current heading
        if (abs(difference) <= 180):
            clockwise = False       # Turn CCW
        else:
            clockwise = True        # Turn CW
            
# Define P & D constants by testing the robot using CW and CCW turns

    if (clockwise):
        # Set PD constants for turning CW
        kP = 0.03
        kD = 0.00

    else:   # P & D values if turning CCW
        kP = 0.10
        kD = 0.02

    # Define max turning velocity and previous variables

    maxVelocity = 50    # Units: %
    previousError = 0.0 # Error from the previoius run of the while loop

    while(True):

        # Proportional term
        turnError = setPoint - inertial_1.heading()


        # Derivative
        derivative = turnError - previousError

        # Stop motors and exit the while() loop if the error and derivative terms are sufficient
        #   small to ensure that the setpoint was reached with minimal oscillation
        #   if you only check the error term, momentum will cause the robot to overhsoot the target
        #   including the derivative will ensure the motors stop after oscillation is eliminated

        if(abs(turnError) < 1 and (abs(derivative < 0.2))):
            stopMotors()        # Stop the motors
            break               # Exit the while() loop
        
        # Calculate the correction term for the motor velocities

        turnCorrection = (kP * turnError) + (kD * derivative)       # P & D control

        # Limit the PD output to remain between -1  and 1
        # This will keep the motor velocity from exceeding its maximum set value
        if (abs(turnCorrection) > 1):
            turnCorrection = 1

        turnVelocity = maxVelocity * turnCorrection

        # SEt motor velocities based on the directionn (CW or CCW) of turn and the PD output

        if(clockwise):  # Turn clockwise
            leftMotor.set_velocity(turnVelocity)            # Turn the left motor forward
            rightMotor.set_velocity(-1 * turnVelocity)      # Turn the right motor in reverse

        else:   # Turn counter-clockwise
            leftMotor.set_velocity(-1 * turnVelocity)            # Turn the left motor forward
            rightMotor.set_velocity(turnVelocity)

        #Spin the motors
        leftMotor.spin(FORWARD)
        rightMotor.spin(FORWARD)

        turnData(turnError, derivative) # Print current ehading value

        #Update previousErro
        previousError = turnError

        wait(20, MSEC)


def liftArm(motorVelocity, liftAngle):
    # Configure the lift motor to hold its position
    liftMotor.set_stopping(HOLD)

    liftMotor.set_velocity(motorVelocity, PERCENT) # Set lift motor velocity

    gearRatio = 5 # 60-T to 12-T 
    motorAngularDisplacement = liftAngle * gearRatio # Calculate the motor axle's angular displacement

    # Spin the motor forward for the given angular displacement
    liftMotor.spin_for(FORWARD, motorAngularDisplacement, DEGREES)
    wait(0.5, SECONDS)


# ---------------------------------------- Define main() -----------------------------------
def main():
    """
    This is the function that will be executed by the brain
    """

    bump()  # Call bump() to begin the program

    inertialCalibration()
    #testInertial()

    # If setpoint is reached without any osciallations, keep kD = 0

    
    
    driveStraight(92,0,50)  # Tune your kP value
    wait (500,MSEC)
    liftArm(50,60)
    wait (500,MSEC)
    driveStraight(30,0,-50)  # Tune your kP value
    wait (500,MSEC)
    pointTurn(90)
    driveStraight(5,0,50)  # Tune your kP value
    wait (500,MSEC)
    liftArm(50,-60)
    #wait(4,SECONDS)
    #driveStraight(84,0,-50)  # Tune your kP value

    

# ----------------------------------------------------------------------------------------

main()