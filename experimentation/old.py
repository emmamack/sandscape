# class Position:
    
#     ticks_per_mm = 50 #TODO
#     ticks_per_deg = 8000 #TODO
    
#     def __init__(self, r=None, theta=None, x=None, y=None, r_ticks=None, theta_ticks=None):
#         if r is not None and theta is not None:
#             self.r = r
#             self.theta = theta
#         elif x is not None and y is not None:
#             self.x = x
#             self.y = y
#         elif r_ticks is not None and theta_ticks is not None:
#             self.r_ticks = r_ticks
#             self.theta_ticks = theta_ticks
#     def rt2xy(self):
#         self.x = self.r * math.cos(math.radians(self.theta))
#         self.y = self.r * math.sin(math.radians(self.theta))
#     def xy2rt(self):
#         self.r = math.sqrt(self.x**2 + self.y**2)
#         self.theta = math.degrees(math.atan2(self.y, self.x))


class Mode:
    """

    Args:
        mode_type (_type_): _description_
            "home" -> marble goes to center at r=0 theta=0
            "spiral" -> marble draws spiral outwards from current location
            "reactive only" -> marble only moves due to touch sensor activation
            "reactive spiral ripple" -> marble does spiral that can be affected by touch sensor activation
    """
    mode_types_list = ["home", "spiral", "reactive only", "reactive spiral ripple"]

    def __init__(self, mode_type):
        self.segment_length = 1
        self.linspeed = 5
        self.density = 10 # used in spirals and ripples
        self.waypoints_xy = []
        self.done = False

        if mode_type == "home":
            self.become_home()
        elif mode_type == "spiral":
            self.become_spiral()
        elif mode_type == "reactive only":
            self.become_reactive()
        elif mode_type == "reactive spiral ripple":
            self.become_reactive_spiral_ripple()

    def all_modes_in_list():
        list = []
        for mode_type in Mode.mode_types_list:
            list.append(Mode(mode_type))
        return list
    
    def become_home(self):
        self.mode_type = "home"
        self.connect_to_nano = False
        self.connect_to_uno = True
        self.check_dials = False
        self.check_touch_sensors = False
        self.check_grbl_status = True
        self.waypoints_xy = [(0,0)]

    def become_spiral(self):
        self.mode_type = "spiral"
        self.connect_to_nano = False
        self.connect_to_uno = True
        self.check_dials = False
        self.check_touch_sensors = False
        self.check_grbl_status = True

    def become_reactive(self):
        self.mode_type = "reactive only"
        self.connect_to_nano = False
        self.connect_to_uno = True
        self.check_dials = False
        self.check_touch_sensors = False
        self.check_grbl_status = True

    def become_reactive_spiral_ripple(self):
        self.mode_type = "reactive spiral ripple"