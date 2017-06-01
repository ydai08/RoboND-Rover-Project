import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check if rover is stuck in a donut
        if Rover.angle_ct > 36:
            # Trigger reverse procedure to escape donut
            Rover.mode = 'reverse'
            # Reset counter
            Rover.angle_ct = 0

        else:
            # Calculate average angle for steering, bias to the left
            steer_ang = np.mean(np.sort(Rover.nav_angles)[int(.75*len(Rover.nav_angles)):]) * 180/np.pi
            # Calculate average angle for retrieving rock, no bias
            rock_ang = np.clip(np.mean(Rover.nav_angles) * 180/np.pi,-15,15)

        # Check for Rover.mode status
        # If mode is forward
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain, navigable terrain looks good
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # Check if the rover is stuck on an obstacle
                if Rover.vel == 0:
                    # Stationary for less than 5 frames, keep counting
                    if Rover.stuck < 5:
                        Rover.stuck += 1
                    # Definitely stuck, execute reversal
                    else:
                        Rover.throttle = -2*Rover.throttle_set
                        Rover.mode = 'reverse'
                        Rover.stuck = 0
                # Check if velocity is below max, then throttle
                elif Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                    # Set steering to left biased angle clipped to the range +/- 15
                    Rover.steer = np.clip(steer_ang, -15, 15)
                else: # Else coast at max velocity
                    Rover.throttle = 0
                    # Set steering to left biased angle clipped to the range +/- 10
                    Rover.steer = np.clip(steer_ang, -10, 10)
                Rover.brake = 0

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # Rover stuck on obstacle, 'reverse' mode
        elif Rover.mode == 'reverse':
            # If rover's moving, change to 'stop' mode to evaluate terrain
            if abs(Rover.vel) > .5:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.mode = 'stop'
                # Reset reverse counter
                Rover.reverse_ct = 0
            # Otherwise continue backing up, steering left
            elif Rover.reverse_ct < 50:
                Rover.throttle = -2*Rover.throttle_set
                Rover.brake = 0
                Rover.steer = 15
                Rover.reverse_ct += 1
            # Backing up did not work, try moving forward and steering right
            elif Rover.reverse_ct < 100:
                Rover.throttle = 2*Rover.throttle_set
                Rover.brake = 0
                Rover.steer = -15
                Rover.reverse_ct += 1
            # Forward throttle did not work, turn to the right in place
            # Quicksand escape
            elif Rover.reverse_ct < 120:
                # Stop throttle
                Rover.throttle = 0
                Rover.brake = 0
                # In place turn to the right to continue left-wall tracking
                Rover.steer = -15
                Rover.reverse_ct += 1
            # Restart search for navigable terrain
            else:
                Rover.mode = 'stop'
                Rover.reverse_ct = 0

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Turn to the right to continue left-wall tracking
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = 3*Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = steer_ang
                    Rover.mode = 'forward'

        # Perception step found a rock sample
        elif Rover.mode == 'rock':
            # Is rover close enough for pickup?
            if Rover.near_sample == 1:
                # If pickup already initiated, return Rover
                if Rover.picking_up:
                    Rover.mode = 'forward'
                    return Rover
                # Otherwise, execute pickup if rover is stopped
                elif Rover.vel < 0.2:
                    Rover.send_pickup = True
                    Rover.mode = 'forward'
                # If rover is still moving, brake
                else:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer= rock_ang
            # Rover still too far for pickup
            # If rover is stuck, look for nav terrain
            elif (Rover.vel <= 0):
                Rover.mode = 'stop'
            # Otherwise move towards the rock
            else:
                # If velocity > 1m/s, slow down
                if Rover.vel > 1:
                    Rover.throttle = -Rover.throttle_set/2
                # Otherwise increase speed
                else:
                    Rover.throttle = Rover.throttle_set
                # Aim for rock
                Rover.steer = rock_ang
                Rover.brake = 0

    # Donut prevention
    # Count consecutive large left turn commands at high speed
    if (Rover.steer > 10) and (Rover.vel >= 1):
        Rover.angle_ct += 1
    else:
        Rover.angle_ct = 0

    # Just to make the rover do something
    # even if no modifications have been made to the code
    #else:
    #    Rover.throttle = 0 #Rover.throttle_set
    #    Rover.steer = 0
    #    Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    #if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
    #    Rover.send_pickup = True

    return Rover
