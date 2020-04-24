from safety_test_generator import TestGenerator
from utils.xml_to_bng_files import convert_test
from fuel_consumption_test_generator import FuelConsumptionTestGenerator

if __name__ == '__main__':
    """
    gen = TestGenerator("easy")      # Parameter is optional, you can also call gen.set_difficulty(str)
    gen.set_files_name("test_case")  # This method call is optional
    
    while True:
        for paths in gen.getTest():
            dbe = paths[0]
            dbc = paths[1]
            convert_test(dbc, dbe)  # Use this function only when DriveBuild doesn't work for you.
            # All your code goes here
            # Be sure to call onTestFinished(sid, vid)
    
    dbe = "D:\\Bachelorarbeit\\code\\scenario\\intersection.dbe.xml"
    dbc = "D:\\Bachelorarbeit\\code\\scenario\\intersection.dbc.xml"
    convert_test(dbc, dbe)
    """

    gen = FuelConsumptionTestGenerator()
    gen._create_urban_environment()