from typing import Union


class LookupTable:
    def __init__(self, **kwargs: Union[list[float], tuple[float]]) -> None:
        if len(kwargs.keys()) < 2:
            raise ValueError("LookupTable: Must have more than one row!")

        key_found = False
        self.rows = {}
        for key, val in kwargs.items():
            # Find length of table based on first row, which should be the key
            if not key_found:
                self.key_name = key

                if val != sorted(val):
                    raise ValueError(
                        "LookupTable: Key values must be in ascending order!"
                    )

                self.key_values = tuple(val)
                self.table_length = len(val)

                if self.table_length < 2:
                    raise ValueError("LookupTable: Should have more than one value!")

                key_found = True
                continue

            # Convert to tuple so values are immutable
            self.rows[key] = tuple(val)

            if len(val) != self.table_length:
                raise IndexError(
                    f"LookupTable: Length of row '{key}' does not match length of key row!"
                )

    def lookup(
        self, lookup_row: str, lookup_value: float, extrapolate: bool = False
    ) -> float:
        # Row name not in table
        if lookup_row not in self.rows:
            return 0.0

        # Go through the key values until we pass value
        start_index = 0
        for i, value in enumerate(self.key_values):
            if value > lookup_value:
                break
            start_index = i

        if extrapolate:
            if lookup_value < self.key_values[0]:
                # Extrapolate below
                difference = self.key_values[1] - self.key_values[0]
                higher_bound = self.key_values[0]
                lower_bound = higher_bound - difference

            elif lookup_value > self.key_values[-1]:
                # Extrapolate above
                difference = self.key_values[-1] - self.key_values[-2]
                lower_bound = self.key_values[-1]
                higher_bound = lower_bound + difference
        else:
            lower_bound = self.key_values[start_index]
            end_index = min(start_index + 1, self.table_length)
            higher_bound = self.key_values[end_index]

        if lower_bound != higher_bound and lower_bound != lookup_value:
            # Linear interpolation based on value between bounds
            rise = lookup_value - lower_bound
            run = higher_bound - lower_bound
            m = rise / run
            return self.rows[lookup_row][start_index] * m
        else:
            # At bound so just grab value
            return self.rows[lookup_row][start_index]
