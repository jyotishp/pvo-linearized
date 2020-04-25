# Linearized Probabilistic Velocity Obstacle

This code linearizes the PVO constraint with respect to all the random
variables.

## Build the code

### Prepare env for the plotting code
```bash
python3 -m venv env
source env/activate
pip install -r requirements.txt
```

### Build the PVO code

```bash
mkdir build
cd build
cmake ..
make
```

## Run the code

In the build directory,

```bash
./mpc_linearized
```

The configuration of agents and the noise parameters are in `main.cpp`.

## Plotting the results

First run the code and save the outputs

```bash
./mpc_linearized > outputs.csv
python ../plot.py outputs.csv
```

The outputs will be saved to `./outputs` directory.

> Make sure that the noise parameters in the CPP code are same as the
> parameters in the python plotting code.
