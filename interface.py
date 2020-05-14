"""Interface for generating test cases."""

from fuel_consumption_test_generator import FuelConsumptionTestGenerator
from time import time

if __name__ == '__main__':
    """
    while True:
        for paths in gen.getTest():
            dbe = paths[0]
            dbc = paths[1]
            #convert_test(dbc, dbe)  # Use this function only when DriveBuild doesn't work for you.
            # All your code goes here
            # Be sure to call onTestFinished(sid, vid)
    """

    gen = FuelConsumptionTestGenerator()

    while True:
        start = time()
        liste = gen.genetic_algorithm()
        end = time()
        print(end - start)



