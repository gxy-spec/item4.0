from src.navigation.approach_region import terminal_geometry


def test_reachable_visible_terminal_is_not_the_target_nearest_lane_point():
    bounds=(-2.4,2.4,-1.1,1.1)
    assert terminal_geometry(0,0,0,[8,0],bounds)[0]
    assert not terminal_geometry(0,0,180,[8,0],bounds)[0]
    assert not terminal_geometry(0,0,0,[0,8],bounds)[0]
    assert not terminal_geometry(0,0,0,[3,0],bounds)[0]
    assert not terminal_geometry(0,0,0,[15,0],bounds)[0]
