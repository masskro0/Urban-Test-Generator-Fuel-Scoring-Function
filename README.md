# Test Generator for Urban-like Scenarios

## Prerequisites
Uage of this project requires BeamNG.research to be installed. A copy can be downloaded from
[here](https://beamng.gmbh/research/). Once downloaded and extracted, it is mandatory to set the environment variable
`BNG_HOME` to the trunk folder.

Additionally, download the Python version 3.7.0 (not tested with other versions) from
[here](https://www.python.org/downloads/release/python-370/) and make sure that Powershell uses the correct version.
You can check it with `python --version` .

## Installation
Right-click on setup.ps1 and choose `Run with Powershell`. This creates the virtual environment and installs the
required packages automatically. You will be asked whether you want to install the additional packages which are needed
for the traffic lights experiment. You can either install them [Y] or not [N].

This script will also run setup.py which creates folders and moves files to the correct directories. This requires to
set the `BNG_HOME` environment variable.

## Usage

Once installed, you can run `main.py` which will generate test cases and run them inside BeamNG.research. You can
configure the test generator with variables provided in `main.py`. `urban_test_generator.py` is the test generator
itself.

To run the fuel-inefficiency experiment, simply run main() in evaluation\fuel_inefficiency\main.py and wait until it
completes.

For the traffic lights experiments, there are several options. The function `create_tests()` will generate new test
cases and moves them to all experiment folders. `collect_images_existing_tests()` runs all test cases of all
experiments and collects images, while `collect_images()` collects images of only one specific test case of one
experiment. Since I already collected images, you can just run `predict_all_images()` which will use the pretrained
traffic light detection system to make predictions and visualize the results. `plot_confusion_matrix()` will plot a
confusion matrix; I included examples for this method.