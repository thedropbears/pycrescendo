from utilities.lookup import LookupTable

test_table = LookupTable(
    angle=[0.0, 10.0, 12.0, 15.0, 20.0],
    distance=[-5.0, -2.0, -2.0, 0.0, 6.0],
    speed=[-3.0, -2.5, -1.0, 0.0, 1.5],
)


def test_zero_gradient():
    assert test_table.lookup("distance", 11.0) == -2.0


def test_not_enough_rows():
    try:
        LookupTable()
    except ValueError:
        pass
    else:
        raise AssertionError

    try:
        LookupTable(key=[0.0, 1.0])
    except ValueError:
        pass
    else:
        raise AssertionError
