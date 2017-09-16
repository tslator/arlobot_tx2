


import rospy

# Wraps Rate so that the hz parameter can be retrieved
class RateWrapper(rospy.Timer.Rate):
    def __init__(self, rate):
        self.super(self, RateWrapper).__init__(rate)
        self._hz = rate

    @attr.getter
    def Rate(self):
        return self._hz