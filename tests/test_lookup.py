from hypothesis import given
from hypothesis.strategies import floats

from utilities.lookup import LookupTable

test_table = LookupTable(
    angle=[0.0, 10.0, 12.0, 15.0, 20.0],
    distance=[-5.0, -2.0, -2.0, 0.0, 6.0],
    speed=[-3.0, -2.5, -1.0, 0.0, 0.5],
)


def test_not_enough_rows():
    does_initialisation_triggers_error(ValueError)
    does_initialisation_triggers_error(ValueError, key=[0.0, 1.0])


def test_not_ascending_key():
    does_initialisation_triggers_error(ValueError, key=[10.0, 0.0], values=[0.0, 1.0])


def test_one_value_table():
    does_initialisation_triggers_error(ValueError, key=[0.0], value=[0.0])


def test_duplicate_keys():
    does_initialisation_triggers_error(ValueError, key=[0.0, 0.0], value=[0.0, 1.0])


def test_multiple_row_lengths():
    does_initialisation_triggers_error(
        IndexError, key=[0.0, 1.0, 2.0], value=[0.0, 1.0]
    )


def test_zero_gradient():
    assert test_table.lookup("distance", 11.0) == -2.0


def test_interpolation():
    assert test_table.lookup("distance", 5.0) == -3.5
    assert test_table.lookup("speed", 17.0) == 0.2


def test_linear_extrapolation():
    assert test_table.lookup("distance", 25.0, True) == 12.0
    assert test_table.lookup("speed", -10.0, True) == -3.5


def test_constant_extrapolation():
    assert test_table.lookup("distance", 25.0) == 6.0
    assert test_table.lookup("speed", -5.0) == -3.0


def test_lookup_at_bounds():
    assert test_table.lookup("distance", 0.0) == -5.0
    assert test_table.lookup("speed", 20.0) == 0.5


@given(lookup_value=floats(0.0, 20.0))
def test_return_between_bounds(lookup_value):
    # Find which two key values the lookup value is between
    for i in range(1, test_table.table_length):
        i1 = i - 1
        i2 = i
        if test_table.key_values[i] > lookup_value:
            break

    row_name = "speed"
    search_row = test_table.rows[row_name]
    found_value = test_table.lookup(row_name, lookup_value)
    assert found_value >= search_row[i1] and found_value <= search_row[i2]


def does_initialisation_triggers_error(error_type: Exception, **kwargs):
    try:
        LookupTable(**kwargs)
    except error_type:
        pass
    else:
        raise AssertionError
