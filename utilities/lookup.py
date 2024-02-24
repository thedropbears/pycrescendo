from collections.abc import Sequence
from utilities.functions import clamp


class LookupTable:
    def __init__(self, key_row: Sequence[float], **kwargs: Sequence[float]) -> None:
        # Find length of table based on first row, which should be the key
        if key_row != sorted(key_row):
            raise ValueError("LookupTable: Key values must be in ascending order!")

        self.key_values = tuple(key_row)
        self.table_length = len(key_row)

        if self.table_length < 2:
            raise ValueError("LookupTable: Should have more than one value!")

        if self.table_length != len(set(key_row)):
            raise ValueError("LookupTable: Should not have duplicate key value!")

        if len(kwargs.keys()) < 1:
            raise ValueError("LookupTable: Must have more than one row!")

        self.rows = {}
        for key, val in kwargs.items():
            # Convert to tuple so values are immutable
            self.rows[key] = tuple(val)

            if len(val) != self.table_length:
                raise IndexError(
                    f"LookupTable: Length of row\
                    {key} does not match length of key row!"
                )

    def lookup(
        self, lookup_row: str, lookup_value: float, linear_extrapolate: bool = False
    ) -> float:
        # Go through the key values until upper bound passes target value
        for i in range(1, self.table_length):
            i1 = i - 1
            i2 = i
            if self.key_values[i] > lookup_value:
                break

        # Linear interpolation based on bounds
        x1 = self.key_values[i1]
        x2 = self.key_values[i2]
        y1 = self.rows[lookup_row][i1]
        y2 = self.rows[lookup_row][i2]
        rise = y2 - y1
        run = x2 - x1

        # Run guaranteed not to equal zero as keys are unique.
        m = rise / run

        if not linear_extrapolate:
            lookup_value = clamp(lookup_value, x1, x2)

        return m * (lookup_value - x1) + (y1)
