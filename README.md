# py2024

The Drop Bears' robot code for FRC 2024

## Setup

### Install Dependencies

```
pip install -r requirements-dev.txt
```

### Setup pre-commit
Pre-commit is setup to automatically run formatters and linters when you commit. It is suggested to install pre-commit via pipx to keep it seperate from robot code dependencies. First [install pipx](https://pipx.pypa.io/stable/installation/).


Install pre-commit with pipx:
```
pipx install pre-commit
```

Setup the pre-commit hooks to run on commit:
```
pre-commit install
```


## Run

### Simulation

```
python -m robotpy sim
```

### Deploy to Robot

Once on robots network

```
python -m robotpy deploy
```

### Test

```
python -m robotpy test
```


## Code Structure

We use RobotPy's Magicbot framework

`robot.py`: Entry point, has mapping from driver inputs to high level robot actions.

`components/`: Abstracts hardware into robot actions.

`controllers/`: Automates robot actions, mostly with state machines.

`autonomous/`: Controls robot during autonomous period.

`ids.py`: Has CAN ids, PH channels and other port numbers.
