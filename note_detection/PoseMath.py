#Credit to team 2996 Cougars Gone Wired

import math

# this program is used to approximate the location of a note relative to the camera

camera_mount_height = 0.72 # this is the height of the camera (center of lens) from the ground in meters
camera_mount_angle_deg = 45 # this is the angle of the camera (center of lens) towards the ground (in degress) down is positive
camera_mount_angle_rad = math.radians(camera_mount_angle_deg)


camera_height_angle_deg = 45.5 # this is the angle of the camera's vertical view in degrees
camera_height_angle_rad = math.radians(camera_height_angle_deg)
camera_height_pixels = 480 # this is the height of the image in pixels
vert_angle_per_pixel = camera_height_angle_rad / camera_height_pixels

camera_width_angle_deg = 53.14 # this is the angle of the camera's horizontal view in degrees
camera_width_angle_rad = math.radians(camera_width_angle_deg)
camera_width_pixels = 640 # this is the width of the image in pixels
horiz_angle_per_pixel = camera_width_angle_rad / camera_width_pixels


def get_y_distance_from_pixels(y_pixels:int):
    distance = - camera_mount_height / math.tan(y_pixels * vert_angle_per_pixel - (camera_height_angle_rad/2) - camera_mount_angle_rad)
    return distance


def get_camera_ray_to_ground(y_pixels:int):
    distance = math.sqrt(camera_mount_height**2 + get_y_distance_from_pixels(y_pixels)**2)
    return distance

def get_x_view_width(y_pixels:int):
    distance = 2 * (get_camera_ray_to_ground(y_pixels) * math.tan(camera_width_angle_rad/2))
    return distance

def get_x_distance_from_pixels(x_pixels:int, y_pixels:int):
    x_view_width = get_x_view_width(y_pixels)
    ray_to_ground = get_camera_ray_to_ground(y_pixels)
    distance = ray_to_ground * math.tan(x_pixels * horiz_angle_per_pixel - (camera_width_angle_rad/2))
    return distance

def get_x_y_distance_from_pixels(x_pixels:int, y_pixels:int):
    return get_x_distance_from_pixels(x_pixels,y_pixels), get_y_distance_from_pixels(y_pixels)

class NotePoseEstimator:
    __mount_height = 0
    __mount_angle_deg = 0
    __mount_angle_rad = 0

    __vert_fov_angle_deg = 0.0
    __vert_fov_angle_rad = 0.0
    __horiz_fov_angle_deg = 0.0
    __horiz_fov_angle_rad = 0.0

    __vert_pixels = 0
    __horiz_pixels = 0
    __vert_angle_per_pixel = 0
    __horiz_angle_per_pixel = 0

    def __init__(self):
        pass

    def get_x_y_distance_from_pixels(self, x_pixels:int, y_pixels:int):
        y_distance = - self.__mount_height / math.tan(y_pixels * self._vert_angle_per_pixel - (self.__vert_fov_angle_rad/2) - self.__mount_angle_rad)
        y_dist_hyp = math.sqrt(self.__mount_height**2 + y_distance**2)
        x_distance = y_dist_hyp * math.tan(x_pixels * self._horiz_angle_per_pixel - (self.__horiz_fov_angle_rad/2))
        return (x_distance, y_distance)
    
    @property
    def camera_mount_height(self):
        return self.__mount_height

    @camera_mount_height.setter
    def camera_mount_height(self, height):
        self.__mount_height = height

    @property
    def camera_mount_angle(self):
        return self.__mount_angle_deg

    @camera_mount_angle.setter
    def camera_mount_angle(self, angle):
        self.__mount_angle_deg = angle
        self.__mount_angle_rad = math.radians(angle)

    @property
    def vertical_fov_angle(self):
        return self.__vert_fov_angle_deg
    
    @vertical_fov_angle.setter
    def vertical_fov_angle(self, angle):
        self.__vert_fov_angle_deg = angle
        self.__vert_fov_angle_rad = math.radians(angle)
    
    @property
    def horizontal_fov_angle(self):
        return self.__horiz_fov_angle_deg
    
    @horizontal_fov_angle.setter
    def horizontal_fov_angle(self, angle):
        self.__horiz_fov_angle_deg = angle
        self.__horiz_fov_angle_rad = math.radians(angle)
    
    @property
    def vertical_pixels(self):
        return self.__vert_pixels
    
    @vertical_pixels.setter
    def vertical_pixels(self, pixels):
        self.__vert_pixels = pixels
    
    @property
    def horizontal_pixels(self):
        return self.__horiz_pixels
    
    @horizontal_pixels.setter
    def horizontal_pixels(self, pixels):
        self.__horiz_pixels = pixels

    @property
    def _horiz_angle_per_pixel(self):
        return self.__horiz_fov_angle_rad / self.__horiz_pixels
    
    @property
    def _vert_angle_per_pixel(self):
        return self.__vert_fov_angle_rad / self.__vert_pixels

note_estimator = NotePoseEstimator()
note_estimator.camera_mount_height = 0.72
note_estimator.camera_mount_angle = 45

note_estimator.vertical_fov_angle = 45.5
note_estimator.vertical_pixels = 480
note_estimator.horizontal_fov_angle = 53.14
note_estimator.horizontal_pixels = 680