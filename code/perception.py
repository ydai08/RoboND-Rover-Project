import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, nav_thresh=(160, 160, 160), rock_thresh=100, obs_thresh=100):
    # Create arrays of zeros same xy size as img, but single channel
    nav_select = np.zeros_like(img[:,:,0])
    obs_select = np.copy(nav_select)
    rock_select = np.copy(nav_select)

    # Filter for navigable terrain
    # RGB > thresh
    nav_filter = (img[:,:,0] > nav_thresh[0]) \
                & (img[:,:,1] > nav_thresh[1]) \
                & (img[:,:,2] > nav_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    nav_select[nav_filter] = 1

    # Filter for yellow rocks
    # Not nav terrain, R > thresh, R&G approx equal, B/R >= 2
    # +1 to all denominators to avoid divide by 0
    rock_filter = ~nav_filter \
                    & (img[:,:,0] > rock_thresh) \
                    & (img[:,:,1]/(img[:,:,0]+1)>0.8) & (img[:,:,1]/(img[:,:,0]+1)<1.2)\
                    & (img[:,:,0]/(img[:,:,2]+1)>2)
    rock_select[rock_filter] = 1

    # Find obstacles
    # Not nav or rock, R+G+B > 0 to avoid mapping out of view, RGB < thresh
    obs_filter = ~nav_filter & ~rock_filter & (img.sum(-1)>0) \
                    & (img[:,:,0] < obs_thresh) \
                    & (img[:,:,1] < obs_thresh) \
                    & (img[:,:,2] < obs_thresh)
    obs_select[obs_filter]=1

    # Return the binary image
    return nav_select, rock_select, obs_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    yaw = np.radians(yaw)
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw) - ypix * np.sin(yaw)
    ypix_rotated = xpix * np.sin(yaw) + ypix * np.cos(yaw)
    # Return the result
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = xpos + xpix_rot/scale
    ypix_translated = ypos + ypix_rot/scale
    # Return the result
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image

    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    nav_select, rock_select, obs_select = color_thresh(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = (obs_select+rock_select) *255
    Rover.vision_image[:,:,1] = rock_select *255
    Rover.vision_image[:,:,2] = nav_select *255

    # When roll and pitch are near zero, map navigable, obstacles, rocks to worldmap
    if (Rover.pitch % 360 < 1) and (Rover.roll % 360 < 1.5):
        def worldcoord(thresh):
            # 5) Convert map image pixel values to rover-centric coords
            # 6) Convert rover-centric pixel values to world coordinates
            scale = 10
            xpix, ypix = rover_coords(thresh)
            xpix_world, ypix_world = pix_to_world(xpix, ypix, Rover.pos[0],
                                                  Rover.pos[1], Rover.yaw,
                                                  Rover.worldmap.shape[0], scale)
            return xpix_world, ypix_world

        # Convert nav, rock, and obs map values to world coordinates
        nav_xpix_world, nav_ypix_world = worldcoord(nav_select)
        rock_xpix_world, rock_ypix_world = worldcoord(rock_select)
        obs_xpix_world, obs_ypix_world = worldcoord(obs_select)

        # 7) Update Rover worldmap (to be displayed on right side of screen)
            # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
            #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
            #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
        Rover.worldmap[nav_ypix_world, nav_xpix_world, 2] += 1
        Rover.worldmap[rock_ypix_world, rock_xpix_world, 1] += 5
        Rover.worldmap[obs_ypix_world, obs_xpix_world, 0] += 1
        Rover.worldmap[:,:,:] = np.clip(Rover.worldmap[:,:,:], 0, 255)

    '''MAPPING ONLY, NO ROCK PICKUP'''
    #angle_select = nav_select

    '''TO ENABLE ROCK PICKUP'''
    # When rover is in view of a rock sample, restrict nav angles to rock pixels
    # only and initiate retrieval mode
    if np.sum(rock_select) > 10:
        angle_select = rock_select
        Rover.mode = 'rock'
    # If rover is in rock mode but does not see the sample, go forward
    elif Rover.mode == 'rock':
        Rover.mode = 'forward'
        angle_select = nav_select
    # When no rock sample is in sight, use all nav pixels to calculate nav angles
    else:
        angle_select = nav_select

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    xpix, ypix = rover_coords(angle_select)
    dist, angles = to_polar_coords(xpix, ypix)
    Rover.nav_dists = dist
    Rover.nav_angles = angles

    return Rover
