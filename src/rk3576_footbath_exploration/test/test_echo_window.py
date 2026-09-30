from rk3576_footbath_exploration.near_obstacle import WindowedConfirmation, ClearConfirmation


def test_far_interrupt_and_duplicates():
    c=WindowedConfirmation()
    assert c.observe('l',1.,(0.,0.)) is None
    for _ in range(10):assert c.observe('l',1.,(0.,0.)) is None
    assert c.observe('l',1.3,None) is None
    assert c.observe('l',1.6,(.005,0.)) is None
    assert c.observe('l',1.9,(0.,0.)) is not None
    assert c.observe('l',2.2,None) is not None
    assert c.observe('l',2.5,None) is None


def test_stale_gap_movement_reset_and_out_of_order():
    c=WindowedConfirmation()
    c.observe('l',1.,(0.,0.));c.observe('l',1.3,(0.,0.))
    assert c.observe('l',2.,(0.,0.)) is None
    c.observe('l',2.3,(0.,0.))
    assert c.observe('l',2.6,(.1,0.)) is None
    assert c.observe('l',2.5,(.1,0.)) is None
    c.observe('l',3.,(0.,0.));c.reset('l')
    assert c.observe('l',3.3,(0.,0.)) is None


def test_far_release_still_needs_three_consecutive_samples():
    c=ClearConfirmation()
    assert not c.observe('side_left',1.,.38)
    assert not c.observe('side_left',1.3,.115)
    assert not c.observe('side_left',1.6,.38)
    assert not c.observe('side_left',1.9,.38)
    assert c.observe('side_left',2.2,.38)


def test_front_release_boundary_is_inclusive_and_side_is_unchanged():
    c=ClearConfirmation()
    assert not c.observe('front',1.,.279999)
    assert not c.observe('front',1.32,.28)
    assert not c.observe('front',1.64,.28)
    assert c.observe('front',1.96,.28)
    for t in (1.,1.32,1.64):assert not c.observe('side_left',t,.12)
