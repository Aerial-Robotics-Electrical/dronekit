# Import DroneKit-Python
from dronekit import connect, VehicleMode
import time

def main():
    # Connect to PixHawk
    vehicle = Initialize("COM7")

    # Print PixHawk attributes
    PrintAttributes(vehicle)

    # Set mode to stabilize
    SetMode(vehicle, "MANUAL")

    # Arm PixHawk
    Arm(vehicle)

    # TAKEOFF!!!
    Takeoff(vehicle, 20)

    # Close the vehicle
    vehicle.close()

def Arm(vehicle):
    print ("\nSet Vehicle.armed=True (currently: %s)" % vehicle.armed)
    vehicle.armed = True
    while not vehicle.armed:
        print ("Waiting for arming...")
        time.sleep(1)
    print ("ARMED")

def Takeoff(vehicle, target_altitude):
    print ("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print ("Altitude: ", vehicle.location.global_relative_frame.alt)
        if (vehicle.location.global_relative_frame.alt >= target_altitude * 0.95):
            print ("Reached target altitude!")
            break
        time.sleep(1)
    
    print("Target height reached")

def SetMode(vehicle, mode):
    print("Setting", mode, "mode...")
    vehicle.mode = VehicleMode(mode)

    while not vehicle.mode.name == mode:
        print ("Waiting for mode change ...")
        time.sleep(1)
    print ("Mode change done: mode is now", vehicle.mode)

def Initialize(port):
    # Connect to serial port.
    print("Connecting to serial port. . .")
    vehicle = connect(port, wait_ready=True, baud=57600)
    print("Connected!")

    # Initialize the vehicle
    while vehicle.mode.name == "INITIALISING":
        print("Waiting for vehicle to initialize")
        time.sleep(1)
    print("Initialized!")

    return vehicle

def PrintAttributes(vehicle):
    print ("Autopilot Firmware version: %s" % vehicle.version)
    print ("Global Location: %s" % vehicle.location.global_frame)
    print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print ("Local Location: %s" % vehicle.location.local_frame)    #NED
    print ("Attitude: %s" % vehicle.attitude)
    print ("Velocity: %s" % vehicle.velocity)
    print ("GPS: %s" % vehicle.gps_0)
    print ("Groundspeed: %s" % vehicle.groundspeed)
    print ("Airspeed: %s" % vehicle.airspeed)
    print ("Gimbal status: %s" % vehicle.gimbal)
    print ("Battery: %s" % vehicle.battery)
    print ("EKF OK?: %s" % vehicle.ekf_ok)
    print ("Last Heartbeat: %s" % vehicle.last_heartbeat)
    print ("Rangefinder: %s" % vehicle.rangefinder)
    print ("Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print ("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print ("Heading: %s" % vehicle.heading)
    print ("Is Armable?: %s" % vehicle.is_armable)
    print ("System status: %s" % vehicle.system_status.state)
    print ("Mode: %s" % vehicle.mode.name)    # settable
    print ("Armed: %s" % vehicle.armed)    # settable

main()