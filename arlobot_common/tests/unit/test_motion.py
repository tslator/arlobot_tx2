from motion import uni2diff

#---------------------------------------------------------------------------------------------------
# Module Under Test: motion.py
# Component Under Test: uni2diff
#---------------------------------------------------------------------------------------------------

def test_WhenInputAllZero_ThenLeftRightAreZero():
    left, right = uni2diff(0.0, 0.0, 0.0, 0.0)

    assert left == 0.0 and right == 0.0

def test_WhenWIsZero_ThenLeftRightAreRatioVOverR():
    left, right = uni2diff(v=1.0, w=0.0, L=1.0, R=1.0)

    assert left == 1.0 and right == 1.0

def test_WhenVIsZero_ThenLeftRightAreOppositeRatioWLOver2R():
    v = 0.0
    w = 1.0
    L = 1.0
    R = 1.0

    left, right = uni2diff(v=v, w=w, L=L, R=R)

    expect_left = -w*L/2*R
    expect_right = -expect_left

    assert left == expect_left and right == expect_right

def test_WhenVIsTwoWIsZeroLIsDontcareRIsTwo_ThenLeftRightAreOne():
    v = 2.0
    w = 0.0
    L = 0.0
    R = 2.0

    left, right = uni2diff(v=v, w=w, L=L, R=R)

    assert left == 1.0 and right == 1.0

#---------------------------------------------------------------------------------------------------
# Module Under Test: motion.py
# Component Under Test: diff2uni
#---------------------------------------------------------------------------------------------------


#---------------------------------------------------------------------------------------------------
# Module Under Test: motion.py
# Component Under Test: ensure_w
#---------------------------------------------------------------------------------------------------

#---------------------------------------------------------------------------------------------------
# Module Under Test: motion.py
# Component Under Test: diff2uni
#---------------------------------------------------------------------------------------------------

#---------------------------------------------------------------------------------------------------
# Module Under Test: motion.py
# Component Under Test: x_dot, y_dot, theta_dot
#---------------------------------------------------------------------------------------------------

#---------------------------------------------------------------------------------------------------
# Module Under Test: motion.py
# Component Under Test: diff2uni
#---------------------------------------------------------------------------------------------------


#---------------------------------------------------------------------------------------------------
# Module Under Test: motion.py
# Component Under Test: uni_max
#---------------------------------------------------------------------------------------------------

#---------------------------------------------------------------------------------------------------
# Module Under Test: motion.py
# Component Under Test: diff_max
#---------------------------------------------------------------------------------------------------

#---------------------------------------------------------------------------------------------------
# Module Under Test: motion.py
# Component Under Test: velocity_smoother
#---------------------------------------------------------------------------------------------------


