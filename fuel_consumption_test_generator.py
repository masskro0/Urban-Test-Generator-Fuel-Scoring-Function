from copy import deepcopy
from random import randint, random, choice

from numpy import asarray, clip, concatenate, arange, linspace, array, linalg
from scipy.interpolate import splev
from shapely import affinity
from shapely.geometry import LineString, shape
from termcolor import colored
from os import path
from glob import glob
from pathlib import Path
from scipy.spatial.distance import euclidean

from utils.utility_functions import convert_points_to_lines, convert_splines_to_lines, get_angle, calc_width, \
    calc_min_max_angles, get_lanes_of_intersection, get_intersection_lines, get_width_lines, \
    get_resize_factor_intersection
from utils.validity_checks import intersection_check_width, intersection_check_last
from utils.xml_creator import build_all_xml
from xml_converter.converter import b_spline

MIN_DEGREES = 70
MAX_DEGREES = 290


def _add_ego_car(individual):
    """Adds the ego car to the criteria xml file. Movement mode can be assigned manually. Each control point is one
    waypoint.
    :param individual: Individual of the population.
    :return: Void.
    """
    # TODO Turning requires lane switch for multiple lanes.
    # TODO Adopt to different number of lanes.
    lanes = individual.get("lanes")
    ego_lanes = individual.get("ego_lanes")
    waypoints = []
    for lane in ego_lanes:
        iterator = 0
        control_points = lanes[lane].get("control_points")
        while iterator < len(control_points):
            waypoint = {"position": control_points[iterator],
                        "tolerance": 2.5,
                        "movementMode": "_BEAMNG"}
            waypoints.append(waypoint)
            iterator += 1
    del waypoints[-1]
    y = (lanes[0].get("left_lanes") + lanes[0].get("right_lanes") - 1) * -2.5 + waypoints[0].get("position")[1]
    init_state = {"position": (lanes[0].get("control_points")[0][0], y),
                  "orientation": 0,
                  "movementMode": "_BEAMNG",
                  "speed": 50}
    model = "ETK800"
    ego = {"id": "ego",
           "init_state": init_state,
           "waypoints": waypoints,
           "model": model}
    participants = [ego]
    individual["participants"] = participants


def _add_parked_cars(individual):
    car_positions = list()
    for idx, lane in enumerate(individual.get("lanes")):
        if lane.get("type") == "intersection":
            continue
        control_points = lane.get("control_points")
        width = lane.get("width")
        rotations = [0, 45, 90]
        rotation = choice(rotations)
        if rotation == 45:
            offset = 3.5
            max_distance = 4
        elif rotation == 90:
            offset = 3
            max_distance = 3
        else:
            offset = 2
            max_distance = 5.5
        right = True if random() >= 0.3 else False
        left = True if random() >= 0.3 else False
        line = LineString(control_points)
        if left:
            left_line = line.parallel_offset(width / 2 + offset, "left")
            coords = b_spline(left_line.coords)
            iterator = 1
            while iterator < len(coords):
                point = coords[iterator]
                if abs(euclidean(point, coords[-1])) < 12:
                    break
                if len(car_positions) == 0 or abs(euclidean(point, car_positions[-1][0])) > max_distance:
                    angle = get_angle((coords[iterator - 1][0] + 5, coords[iterator - 1][1]),
                                      coords[iterator - 1], point) - rotation
                    car_positions.append((point, angle))
                iterator += 1
        if right:
            right_line = line.parallel_offset(width / 2 + offset, "right")
            coords = right_line.coords[::-1]
            coords = b_spline(coords)
            iterator = 1
            while iterator < len(coords):
                point = coords[iterator]
                if abs(euclidean(point, coords[-1])) < 12:
                    break
                if len(car_positions) == 0 or abs(euclidean(point, car_positions[-1][0])) > max_distance:
                    angle = get_angle((coords[iterator - 1][0] + 5, coords[iterator - 1][1]),
                                       coords[iterator - 1], point) + 180 - rotation
                    car_positions.append((point, angle))
                iterator += 1
    parked_cars = list()
    for position in car_positions:
        if random() <= 0.4:
            continue
        parked_cars.append({"name": "golf", "position": position[0], "zRot": position[1]})
    individual["obstacles"].extend(parked_cars)


def _merge_lanes(population):
    """Merge lanes for each individual which will be driven by the ego car.
    :param population: Population list.
    :return: Population with merged lanes.
    """
    for individual in population:
        iterator = 1
        lanes = individual.get("lanes")
        ego_lanes = individual.get("ego_lanes")
        new_lane_list = [lanes[0]]
        while iterator < len(lanes):
            if iterator in ego_lanes and (iterator - 1) in ego_lanes:
                new_lane_list[-1].get("control_points").extend(lanes[iterator].get("control_points"))
            else:
                new_lane_list.append(lanes[iterator])
            iterator += 1
        individual["lanes"] = new_lane_list
    return population


def _add_traffic_signs(last_point, intersection_point, current_left_lanes, current_right_lanes,
                       new_left_lanes, new_right_lanes, layout, number_of_ways, direction, left_point,
                       straight_point, right_point, width, width_opposite):
    def opposite_direction(my_point, my_right_point):
        line = LineString([(intersection_point[0], intersection_point[1]),
                           (my_point[0], my_point[1])])
        my_z_rot = int(round(get_angle(temp_point, line.coords[0], line.coords[1]))) + 180
        angle = int(round(get_angle((my_right_point[0], my_right_point[1]),
                                    line.coords[0], line.coords[1])))
        offset = 0.1 if angle <= 270 else ((angle - 270) / 10 + 0.2) * 1.3
        fac = (old_width * (current_left_lanes + current_right_lanes) / 2 + offset) / line.length
        vector = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        vector = affinity.rotate(vector, -90, vector.coords[1])
        fac = (new_width * (new_left_lanes + new_right_lanes) / 2 + 0.2) / vector.length
        vector = affinity.scale(vector, xfact=fac, yfact=fac, origin=vector.coords[1])
        my_position = vector.coords[0]
        if sign_on_my_lane == "stopsign":
            obstacles.append({"name": "prioritysign", "position": my_position, "zRot": my_z_rot})
        else:
            if new_left_lanes + new_right_lanes == 2:
                obstacles.append({"name": "trafficlightsingle", "position": my_position, "zRot": my_z_rot})
            else:
                obstacles.append({"name": "trafficlightdouble", "position": my_position, "zRot": my_z_rot})

    def my_direction(my_point, my_right_point):
        line = LineString([(intersection_point[0], intersection_point[1]),
                           (my_point[0], my_point[1])])
        my_z_rot = int(round(get_angle(temp_point, line.coords[0], line.coords[1]))) + 180
        angle = int(round(get_angle((my_right_point[0], my_right_point[1]), line.coords[0], line.coords[1])))
        offset = 0.1 if angle <= 270 else ((angle - 270) / 10 + 0.2) * 1.3
        fac = (new_width * (new_left_lanes + new_right_lanes) / 2 + offset) / line.length
        vector = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        vector = affinity.rotate(vector, -90, vector.coords[1])
        fac = (old_width * (current_left_lanes + current_right_lanes) / 2 + 0.2) / vector.length
        vector = affinity.scale(vector, xfact=fac, yfact=fac, origin=vector.coords[1])
        my_position = vector.coords[0]
        return my_position, my_z_rot

    # Calculate traffic sign position.
    old_width = width / (current_left_lanes + current_right_lanes)
    new_width = width_opposite / (new_left_lanes + new_right_lanes)
    obstacles = list()
    temp_point = (intersection_point[0] + 5, intersection_point[1])

    # Bottom direction.
    position, z_rot = my_direction(last_point, right_point)
    if current_left_lanes + current_right_lanes == 2:
        if random() <= 0.5:
            obstacles.append({"name": "stopsign", "position": position, "zRot": z_rot})
        else:
            obstacles.append({"name": "trafficlightsingle", "position": position, "zRot": z_rot})
    else:
        obstacles.append({"name": "trafficlightdouble", "position": position, "zRot": z_rot})
    sign_on_my_lane = obstacles[0].get("name")

    # Left direction.
    if number_of_ways == 4 or direction == "left" or layout == "left":
        opposite_direction(left_point, last_point)

    # Top direction.
    if number_of_ways == 4 or direction == "straight" or layout == "straight":
        position, z_rot = my_direction(straight_point, left_point)
        obstacles.append({"name": sign_on_my_lane, "position": position, "zRot": z_rot})

    # Right direction.
    if number_of_ways == 4 or direction == "right" or layout == "right":
        opposite_direction(right_point, straight_point)

    return obstacles


class FuelConsumptionTestGenerator:

    def __init__(self):
        self.files_name = "urban"
        self.SPLINE_DEGREE = 3  # Sharpness of curves
        self.MAX_TRIES = 60  # Maximum number of invalid generated points/segments
        self.POPULATION_SIZE = 1  # Minimum number of generated roads for each generation
        self.NUMBER_ELITES = 2  # Number of best kept roads
        self.MIN_SEGMENT_LENGTH = 15  # Minimum length of a road segment
        self.MAX_SEGMENT_LENGTH = 30  # Maximum length of a road segment
        self.MIN_NODES = 6  # Minimum number of control points for each road
        self.MAX_NODES = 12  # Maximum number of control points for each road
        self.population_list = []
        self.intersection_length = 50
        self.opposite_lane = 30
        self.intersecting_length = 20
        self.MAX_LEFT_LANES = 2
        self.MAX_RIGHT_LANES = 2
        self.MAX_WIDTH = 5
        print(colored("##################", attrs=["bold"]))
        print(colored("##################", "red", attrs=["bold"]))
        print(colored("##################", "yellow", attrs=["bold"]))

    def _bspline(self, lanes):
        """Calculate {@code samples} samples on a bspline. This is the road representation function.
        :param lanes: List of lanes.
        :return: List of arrays with samples, representing a bspline of the given control points of the lanes.
        """
        splined_list = []
        for lane in lanes:
            samples = lane.get("samples")
            # Calculate splines for each lane.
            point_list = asarray(lane.get("control_points"))
            count = len(point_list)
            degree = clip(self.SPLINE_DEGREE, 1, count - 1)

            # Calculate knot vector.
            kv = concatenate(([0] * degree, arange(count - degree + 1), [count - degree] * degree))

            # Calculate query range.
            u = linspace(False, (count - degree), samples)

            # Calculate result.
            splined_list.append({"control_points": array(splev(u, (kv, point_list.T, degree))).T,
                                 "width": lane.get("width")})
        return splined_list

    def _add_segment(self, last_point, penultimate_point=None):
        """Generates a new random point within a given range.
        :param last_point: Last point of the control point list as dict type.
        :param penultimate_point: Point before the last point as dict type.
        :return: A new random point as dict type.
        """
        last_point_tmp = (last_point[0], last_point[1])
        last_point_tmp = asarray(last_point_tmp)
        x_min = int(round(last_point[0] - self.MAX_SEGMENT_LENGTH))
        x_max = int(round(last_point[0] + self.MAX_SEGMENT_LENGTH))
        y_min = int(round(last_point[1] - self.MAX_SEGMENT_LENGTH))
        y_max = int(round(last_point[1] + self.MAX_SEGMENT_LENGTH))
        while True:
            x_pos = randint(x_min, x_max)
            y_pos = randint(y_min, y_max)
            point = (x_pos, y_pos)
            dist = linalg.norm(asarray(point) - last_point_tmp)
            deg = None
            if penultimate_point is not None:
                deg = get_angle((penultimate_point[0], penultimate_point[0]),
                                (last_point[0], last_point[1]),
                                point)
            if self.MAX_SEGMENT_LENGTH >= dist >= self.MIN_SEGMENT_LENGTH:
                if penultimate_point is not None:
                    if MIN_DEGREES <= deg <= MAX_DEGREES:
                        return point
                else:
                    return point

    def _create_urban_environment(self):
        global MIN_DEGREES, MAX_DEGREES
        print(colored("Creating urban scenario...", "grey", attrs=['bold']))
        p0 = (1, 0)
        p1 = (30, 0)
        p2 = (45, 0)
        left_lanes = randint(1, self.MAX_LEFT_LANES)
        right_lanes = randint(1, self.MAX_RIGHT_LANES)
        lanes = [{"control_points": [p0, p1, p2], "width": calc_width(left_lanes, right_lanes),
                  "left_lanes": left_lanes, "right_lanes": right_lanes, "samples": 100, "type": "normal"}]
        ego_lanes = [0]
        intersection_lanes = []
        obstacles = []
        tries = 0
        lane_index = 0
        number_of_pieces = 3
        one_intersection = False
        intersection_possible = True
        intersection_probability = 0.25
        lines_of_roads = convert_points_to_lines(lanes)
        last_point = p2
        while number_of_pieces <= self.MAX_NODES and tries <= self.MAX_TRIES:
            control_points = lanes[lane_index].get("control_points")
            if intersection_possible and ((number_of_pieces == self.MAX_NODES - 1 and not one_intersection)
                                          or random() <= intersection_probability) and len(control_points) > 1:
                # Add intersection, if possible.
                intersection = self._create_intersection(control_points[-1], control_points[-2])
                intersection_items = get_lanes_of_intersection(intersection, control_points[-1],
                                                               lanes[lane_index].get("width"),
                                                               lanes[lane_index].get("left_lanes"),
                                                               lanes[lane_index].get("right_lanes"), lane_index)
                new_line, new_lane_line = get_intersection_lines(control_points[-1], intersection)
                temp_list = deepcopy(lanes)
                temp_list.extend(intersection_items.get("lanes"))
                temp_list = self._bspline(temp_list)
                control_points_lines = convert_splines_to_lines(temp_list)
                width_lines = get_width_lines(temp_list)
                intersection_lanes_temp = deepcopy(intersection_lanes)
                intersection_lanes_temp.extend(intersection_items.get("intersection_lanes"))
                if not intersection_check_last(lines_of_roads, new_line) \
                        and not intersection_check_last(lines_of_roads, new_lane_line) \
                        and not intersection_check_width(width_lines, control_points_lines, intersection_lanes_temp):
                    left_lanes = intersection_items.get("left_lanes")
                    right_lanes = intersection_items.get("right_lanes")
                    obstacles.extend(_add_traffic_signs(control_points[-1], intersection.get("intersection_point"),
                                                        lanes[lane_index].get("left_lanes"),
                                                        lanes[lane_index].get("right_lanes"),
                                                        left_lanes, right_lanes, intersection.get("layout"),
                                                        intersection.get("number_of_ways"),
                                                        intersection.get("direction"),
                                                        intersection.get("left_point"),
                                                        intersection.get("straight_point"),
                                                        intersection.get("right_point"),
                                                        lanes[lane_index].get("width"),
                                                        intersection.get("new_width")))
                    lanes.extend(intersection_items.get("lanes"))
                    ego_lanes.extend(intersection_items.get("ego_lanes"))
                    last_point = intersection_items.get("last_point")
                    intersection_lanes.extend(intersection_items.get("intersection_lanes"))
                    lane_index = intersection_items.get("lane_index")
                    MIN_DEGREES, MAX_DEGREES = calc_min_max_angles(left_lanes + right_lanes)
                    lines_of_roads = convert_points_to_lines(lanes)
                    number_of_pieces += 1
                    one_intersection = True
                else:
                    intersection_possible = False
            # Add segment, if possible.
            control_points = lanes[lane_index].get("control_points")
            if len(control_points) == 1:
                new_point = self._add_segment(control_points[0])
            else:
                new_point = self._add_segment(control_points[-1], control_points[-2])
            new_line = LineString([(control_points[-1][0], control_points[-1][1]),
                                   (new_point[0], new_point[1])])
            temp_list = deepcopy(lanes)
            temp_list[lane_index].get("control_points").append(new_point)
            temp_list = self._bspline(temp_list)
            control_points_lines = convert_splines_to_lines(temp_list)
            width_lines = get_width_lines(temp_list)
            if not intersection_check_last(lines_of_roads, new_line, max_intersections=0) \
                    and not intersection_check_width(width_lines, control_points_lines, intersection_lanes):
                lanes[lane_index].get("control_points").append(new_point)
                intersection_possible = True
                tries = 0
                number_of_pieces += 1
                lines_of_roads = convert_points_to_lines(lanes)
                last_point = new_point
            else:
                tries += 1
        if number_of_pieces >= self.MIN_NODES and one_intersection:
            print(colored("Finished creating urban scenario!", "grey", attrs=['bold']))
            return {"lanes": lanes, "success_point": last_point, "ego_lanes": ego_lanes, "obstacles": obstacles}
        else:
            print(colored("Couldn't create a valid road network. Restarting...", "grey", attrs=['bold']))

    def _repair(self):
        pass

    def _add_traffic(self):
        pass

    def _create_intersection(self, last_point, penultimate_point):
        layout = None
        random_number = random()
        if random_number <= 0.33:
            direction = "straight"
        elif 0.33 < random_number <= 0.66:
            direction = "left"
        else:
            direction = "right"
        if random() <= 0.5:
            number_of_ways = 4
        else:
            number_of_ways = 3
            if direction == "straight":
                if random() <= 0.5:
                    layout = "left"
                else:
                    layout = "right"
            elif direction == "left":
                if random() <= 0.5:
                    layout = "straight"
                else:
                    layout = "right"
            else:
                if random() <= 0.5:
                    layout = "straight"
                else:
                    layout = "left"
        line = LineString([(penultimate_point[0], penultimate_point[1]),
                           (last_point[0], last_point[1])])
        fac = get_resize_factor_intersection(line.length, self.intersection_length)
        line_intersection = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        straight_point = list(shape(line_intersection).coords)[1]
        fac = get_resize_factor_intersection(line.length, self.intersecting_length)
        line_intersection = affinity.scale(line, xfact=fac, yfact=fac, origin=line.coords[0])
        intersection_point = list(shape(line_intersection).coords)[1]
        line = LineString([(intersection_point[0], intersection_point[1]),
                           (line.coords[1][0], line.coords[1][1])])
        fac = get_resize_factor_intersection(line.length, self.opposite_lane)

        # Right turn.
        right_turn_angle = randint(-110, -70)

        line_rot1 = affinity.rotate(line, right_turn_angle, line.coords[0])
        line_rot1 = affinity.scale(line_rot1, xfact=fac, yfact=fac,
                                   origin=line_rot1.coords[0])

        # Left turn.
        left_turn_angle = randint(70, 110)
        line_rot2 = affinity.rotate(line, left_turn_angle, line.coords[0])
        line_rot2 = affinity.scale(line_rot2, xfact=fac, yfact=fac,
                                   origin=line_rot2.coords[0])
        p1 = (list(shape(line_rot1).coords)[1][0], list(shape(line_rot1).coords)[1][1])
        p2 = (list(shape(line_rot2).coords)[1][0], list(shape(line_rot2).coords)[1][1])
        left_lanes = randint(1, self.MAX_LEFT_LANES)
        right_lanes = randint(1, self.MAX_RIGHT_LANES)
        return {"intersection_point": intersection_point,
                "straight_point": straight_point,
                "left_point": p1,
                "right_point": p2,
                "direction": direction,
                "number_of_ways": number_of_ways,
                "layout": layout,
                "new_left_lanes": left_lanes,
                "new_right_lanes": right_lanes,
                "new_width": calc_width(left_lanes, right_lanes)}

    def _spline_population(self, population):
        """Converts the control points list of every individual to a bspline
         list and adds the width parameter as well as the ego car.
        :param population: List of individuals.
        :return: List of individuals with bsplined control points.
        """
        for individual in population:
            splined_list = self._bspline(individual.get("lanes"))
            iterator = 0
            while iterator < len(splined_list):
                lane = splined_list[iterator]
                individual.get("lanes")[iterator]["control_points"] = lane.get("control_points").tolist()
                iterator += 1
            _add_parked_cars(individual)
            _add_ego_car(individual)
        return population

    def _create_start_population(self):
        """Creates and returns an initial population."""
        startpop = []
        iterator = 0
        while len(startpop) < self.POPULATION_SIZE:
            urban = self._create_urban_environment()
            if urban is not None:
                individual = {"lanes": urban.get("lanes"),
                              "type": "urban",
                              "file_name": self.files_name,
                              "score": 0,
                              "obstacles": urban.get("obstacles"),
                              "success_point": urban.get("success_point"),
                              "ego_lanes": urban.get("ego_lanes")}
                startpop.append(individual)
                iterator += 1
        return startpop

    def genetic_algorithm(self):
        """
        if len(self.population_list) == 0:
            self.population_list = self._create_start_population()
        print(colored("Population finished.", "grey", attrs=['bold']))
        temp_list = deepcopy(self.population_list)
        # plot_all(temp_list)
        """
        individual = {'lanes': [{'control_points': [[1.0, 0.0], [5.232195866085265, -0.00040794985188414423], [9.152026677687328, -0.003263598815073154], [12.776175436643756, -0.0110146460008719], [16.121325144792138, -0.02610879052058523], [19.20415880397005, -0.05099373148551806], [22.041359416015066, -0.0881171680069752], [24.649609982764765, -0.1399267991962615], [27.04559350605673, -0.20887032416468185], [29.245992987728524, -0.2973954420235411], [31.267491429617742, -0.40794985188414445], [33.12677183356196, -0.5429812528577962], [34.84051720139875, -0.7049373440558016], [36.425410534965685, -0.896265824589465], [37.89813483610035, -1.119414393570092], [39.27537310664032, -1.3768307501089867], [40.573808348423185, -1.6709625933174548], [41.8101235632865, -2.0042576223068007], [43.001001753067854, -2.379163536188329], [44.163125919604845, -2.7981280340733456], [45.31315518893317, -3.2635900548868615], [46.46268701709474, -3.7761313780597523], [47.612025605852764, -4.332190214906265], [48.75994710565162, -4.927644124817882], [49.90522766693565, -5.558370667186095], [51.046643440149204, -6.220247401402384], [52.18297057573662, -6.909151886858243], [53.31298522414224, -7.620961682945152], [54.43546353581044, -8.351554349054602], [55.549181661185536, -9.09680744457808], [56.6529157507119, -9.85259852890707], [57.74544195483386, -10.614805161433054], [58.82553642399577, -11.379304901547533], [59.89197530864198, -12.141975308641975], [60.94353475921683, -12.89869394210788], [61.978990926164684, -13.645338361336728], [62.997119959929876, -14.377786125720009], [63.996698010956756, -15.09191479464921], [64.97650122968967, -15.783601927515813], [65.93530576657298, -16.448725083711313], [66.87192556108994, -17.08319892459266], [67.78664360161144, -17.68438045042473], [68.68165122297354, -18.25150031072896], [69.55926729801845, -18.783914374160272], [70.42181069958849, -19.280978509373572], [71.27160030052593, -19.74204858502379], [72.11095497367306, -20.16648046976585], [72.9421935918722, -20.55363003225466], [73.76763502796561, -20.902853141145155], [74.58959815479558, -21.213505665092242], [75.4104018452044, -21.484943472750842], [76.2323649720344, -21.71652243277588], [77.05780640812779, -21.90759841382227], [77.88904502632694, -22.05752728454494], [78.72839969947408, -22.165664913598796], [79.57818930041152, -22.231367169638776], [80.44073270198155, -22.25398992131979], [81.31834877702646, -22.23288903729675], [82.21335639838853, -22.167420386224588], [83.12807443891006, -22.05693983675822], [84.06454118781944, -21.901153407351757], [85.02059657212193, -21.705006394935996], [85.99084861127, -21.47747730338793], [86.96982218883046, -21.22764838467318], [87.9520421883701, -20.964601890757383], [88.9320334934558, -20.697420073606178], [89.90432098765434, -20.435185185185187], [90.8634295545325, -20.186979477460042], [91.80388407765714, -19.961885202396374], [92.72020944059513, -19.76898461195982], [93.60693052691319, -19.617359958116], [94.45857222017817, -19.51609349283056], [95.26965940395691, -19.47426746806912], [96.03471696181624, -19.500964135797318], [96.74826977732293, -19.605265747980773], [97.40484273404384, -19.796254556585133], [97.99896071554575, -20.08301281357602], [98.52514860539551, -20.474622770919066], [98.97793128715995, -20.980166680579906], [99.35183364440583, -21.60872679452416], [99.6430405301184, -22.36813214277249], [99.86000500876533, -23.256949661908337], [100.01667879351278, -24.26959499082241], [100.12703953454897, -25.400464186812524], [100.20506488206212, -26.643953307176442], [100.26473248624049, -27.994458409212015], [100.32001999727233, -29.446375550216995], [100.38490506534583, -30.994100787489206], [100.47336534064928, -32.632030178326474], [100.59937847337093, -34.35455978002656], [100.77692211369899, -36.15608564988729], [101.01997391182168, -38.031003845206484], [101.34251151792729, -39.97371042328189], [101.75851258220405, -41.97860144141137], [102.28195475484017, -44.04007295689266], [102.92681568602393, -46.15252102702361], [103.70707302594354, -48.31034170910204], [104.63670442478727, -50.5079310604257], [105.72968753274336, -52.73968513829244], [107.0, -55.0]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [[107.0, -55.0], [107.42874646285628, -55.71457743809379], [107.85749292571253, -56.42915487618757], [108.28623938856882, -57.14373231428136], [108.7149858514251, -57.85830975237515], [109.14373231428137, -58.57288719046893], [109.57247877713763, -59.28746462856272], [110.00122523999391, -60.00204206665651], [110.42997170285018, -60.7166195047503], [110.85871816570645, -61.43119694284408], [111.28746462856273, -62.145774380937866], [111.716211091419, -62.860351819031656], [112.14495755427527, -63.57492925712544], [112.57370401713155, -64.28950669521922], [113.00245047998781, -65.00408413331301], [113.43119694284408, -65.7186615714068], [113.85994340570035, -66.4332390095006], [114.28868986855663, -67.14781644759438], [114.7174363314129, -67.86239388568816], [115.14618279426917, -68.57697132378195], [115.57492925712545, -69.29154876187573], [116.00367571998171, -70.00612619996951], [116.43242218283798, -70.7207036380633], [116.86116864569426, -71.4352810761571], [117.28991510855053, -72.14985851425088]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [[161.45213454869508, -48.70421128239744], [159.6120420720224, -49.681113250391334], [157.7719495953497, -50.658015218385216], [155.93185711867702, -51.63491718637912], [154.09176464200434, -52.611819154373016], [152.25167216533166, -53.588721122366906], [150.41157968865895, -54.5656230903608], [148.57148721198627, -55.54252505835469], [146.7313947353136, -56.51942702634859], [144.89130225864088, -57.49632899434248], [143.0512097819682, -58.47323096233637], [141.2111173052955, -59.45013293033027], [139.3710248286228, -60.42703489832416], [137.53093235195013, -61.40393686631805], [135.69083987527745, -62.380838834311945], [133.85074739860474, -63.35774080230584], [132.01065492193203, -64.33464277029974], [130.17056244525935, -65.31154473829362], [128.33046996858667, -66.28844670628752], [126.49037749191397, -67.26534867428141], [124.6502850152413, -68.24225064227531], [122.8101925385686, -69.2191526102692], [120.9701000618959, -70.19605457826309], [119.13000758522323, -71.17295654625698], [117.28991510855053, -72.14985851425088]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [[117.28991510855053, -72.14985851425088], [115.54196735590286, -73.28341770185057], [113.79401960325515, -74.41697688945025], [112.04607185060748, -75.55053607704995], [110.2981240979598, -76.68409526464964], [108.55017634531211, -77.81765445224934], [106.80222859266442, -78.95121363984903], [105.05428084001673, -80.08477282744872], [103.30633308736904, -81.21833201504842], [101.55838533472135, -82.3518912026481], [99.81043758207367, -83.48545039024779], [98.062489829426, -84.61900957784749], [96.3145420767783, -85.75256876544718], [94.5665943241306, -86.88612795304687], [92.81864657148293, -88.01968714064657], [91.07069881883524, -89.15324632824625], [89.32275106618755, -90.28680551584594], [87.57480331353986, -91.42036470344564], [85.82685556089217, -92.55392389104533], [84.0789078082445, -93.68748307864503], [82.3309600555968, -94.82104226624472], [80.58301230294911, -95.9546014538444], [78.83506455030144, -97.08816064144409], [77.08711679765375, -98.22171982904378], [75.33916904500606, -99.35527901664348]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [[117.28991510855053, -72.14985851425088], [117.93303480283495, -73.22172467139156], [118.57615449711933, -74.29359082853223], [119.21927419140376, -75.36545698567292], [119.86239388568816, -76.4373231428136], [120.50551357997257, -77.50918929995429], [121.14863327425698, -78.58105545709496], [121.7917529685414, -79.65292161423564], [122.4348726628258, -80.72478777137633], [123.0779923571102, -81.796653928517], [123.72111205139461, -82.86852008565768], [124.36423174567904, -83.94038624279835], [125.00735143996343, -85.01225239993903], [125.65047113424782, -86.08411855707972], [126.29359082853225, -87.1559847142204], [126.93671052281665, -88.22785087136108], [127.57983021710106, -89.29971702850175], [128.22294991138548, -90.37158318564244], [128.86606960566988, -91.44344934278311], [129.50918929995427, -92.5153154999238], [130.1523089942387, -93.58718165706448], [130.7954286885231, -94.65904781420515], [131.4385483828075, -95.73091397134583], [132.08166807709193, -96.8027801284865], [132.72478777137633, -97.8746462856272]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [[132.72478777137633, -97.8746462856272], [133.03638306554706, -98.57657493912912], [133.34844231485894, -99.28072176663298], [133.66098577211397, -99.98693295176092], [133.97403369011386, -100.6950546781349], [134.28760632166046, -101.40493312937701], [134.6017239195557, -102.11641448910929], [134.9164067366013, -102.82934494095373], [135.2316750255992, -103.54357066853245], [135.54754903935108, -104.25893785546744], [135.864049030659, -104.97529268538082], [136.18119525232464, -105.69248134189455], [136.49900795714993, -106.41035000863067], [136.81750739793668, -107.12874486921135], [137.1367138274867, -107.84751210725848], [137.45664749860182, -108.5664979063942], [137.77732866408388, -109.2855484502405], [138.09877757673482, -110.00450992241949], [138.42101448935637, -110.72322850655311], [138.74405965475043, -111.44155038626354], [139.0679333257188, -112.15932174517275], [139.39265575506332, -112.87638876690272], [139.71824719558586, -113.59259763507563], [140.0447279000882, -114.3077945333134], [140.37211812137227, -115.0218256452382], [140.7004381122398, -115.73453715447194], [141.0297081254928, -116.4457752446368], [141.35994841393293, -117.15538609935473], [141.69117923036205, -117.86321590224779], [142.0234208275821, -118.56911083693802], [142.35669345839486, -119.2729170870475], [142.69101737560214, -119.97448083619825], [143.02641283200583, -120.67364826801231], [143.3629000804078, -121.37026556611177], [143.70049937360983, -122.06417891411863], [144.03923096441372, -122.7552344956549], [144.37911510562142, -123.44327849434269], [144.72017205003468, -124.12815709380405], [145.06242205045533, -124.80971647766094], [145.40588535968524, -125.4878028295355], [145.75058223052636, -126.16226233304972], [146.09653291578041, -126.83294117182571], [146.44375766824916, -127.49968552948538], [146.7922767407346, -128.1623415896509], [147.1421103860385, -128.82075553594433], [147.49327885696266, -129.4747735519876], [147.84580240630893, -130.12424182140276], [148.19970128687928, -130.76900652781202], [148.55499575147542, -131.40891385483727], [148.91170605289918, -132.04380998610054], [149.26985244395252, -132.67354110522402], [149.62945517743714, -133.29795339582964], [149.99053450615492, -133.91689304153945], [150.35311068290773, -134.53020622597552], [150.71720396049741, -135.1377391327599], [151.08283459172577, -135.7393379455146], [151.4500228293947, -136.3348488478617], [151.81878892630596, -136.92411802342326], [152.18915313526142, -137.50699165582125], [152.56113570906297, -138.0833159286778], [152.9347569005124, -138.65293702561493], [153.31003696241157, -139.21570113025464], [153.6869961475623, -139.771454426219], [154.06565470876643, -140.32004309713008], [154.4460328988258, -140.8613133266099], [154.82815097054225, -141.39511129828054], [155.21202917671764, -141.92128319576398], [155.5976877701538, -142.4396752026823], [155.98514700365254, -142.95013350265756], [156.37442713001576, -143.45250427931177], [156.76554840204523, -143.946633716267], [157.15853107254284, -144.4323679971453], [157.5533953943104, -144.9095533055687], [157.9501616201498, -145.37803582515926], [158.34885000286278, -145.837661739539], [158.74948079525126, -146.28827723232996], [159.15207425011707, -146.7297284871542], [159.55665062026202, -147.16186168763377], [159.96323015848796, -147.58452301739072], [160.37183311759674, -147.99755866004705], [160.78247975039022, -148.40081479922492], [161.19519030967018, -148.79413761854624], [161.6099850482385, -149.1773733016331], [162.026884218897, -149.55036803210757], [162.44590807444757, -149.91296799359165], [162.867076867692, -150.26501936970743], [163.2904108514321, -150.60636834407694], [163.71593027846978, -150.93686110032223], [164.14365540160682, -151.25634382206533], [164.57360647364513, -151.5646626929283], [165.00580374738647, -151.86166389653314], [165.4402674756327, -152.14719361650197], [165.8770179111857, -152.42109803645678], [166.31607530684727, -152.68322334001962], [166.7574599154193, -152.93341571081254], [167.20119198970357, -153.1715213324576], [167.64729178250192, -153.39738638857685], [168.09577954661623, -153.61085706279226], [168.5466755348483, -153.81177953872597], [169.0, -154.0]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}, {'control_points': [[169.0, -154.0], [169.77373057573772, -154.3094922302951], [170.54746115147543, -154.61898446059016], [171.32119172721315, -154.92847669088525], [172.09492230295086, -155.23796892118037], [172.86865287868858, -155.54746115147543], [173.6423834544263, -155.85695338177052], [174.416114030164, -156.16644561206562], [175.18984460590173, -156.4759378423607], [175.96357518163944, -156.7854300726558], [176.73730575737716, -157.09492230295086], [177.51103633311487, -157.40441453324595], [178.2847669088526, -157.71390676354105], [179.0584974845903, -158.02339899383614], [179.83222806032802, -158.33289122413123], [180.60595863606574, -158.6423834544263], [181.37968921180345, -158.9518756847214], [182.15341978754117, -159.26136791501648], [182.92715036327888, -159.57086014531157], [183.7008809390166, -159.88035237560666], [184.47461151475432, -160.18984460590175], [185.24834209049203, -160.49933683619685], [186.02207266622975, -160.8088290664919], [186.79580324196746, -161.118321296787], [187.56953381770518, -161.4278135270821]], 'width': 15, 'left_lanes': 2, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [[193.4909535876314, -111.77968377471046], [193.24422776388448, -113.84835584772594], [192.99750194013754, -115.91702792074143], [192.75077611639063, -117.98569999375691], [192.50405029264368, -120.0543720667724], [192.2573244688968, -122.12304413978788], [192.01059864514986, -124.19171621280337], [191.7638728214029, -126.26038828581885], [191.51714699765603, -128.32906035883434], [191.27042117390906, -130.39773243184982], [191.02369535016214, -132.4664045048653], [190.77696952641523, -134.5350765778808], [190.5302437026683, -136.60374865089628], [190.28351787892137, -138.67242072391176], [190.03679205517443, -140.74109279692726], [189.79006623142752, -142.80976486994274], [189.5433404076806, -144.87843694295822], [189.29661458393366, -146.9471090159737], [189.04988876018672, -149.01578108898917], [188.8031629364398, -151.08445316200468], [188.5564371126929, -153.15312523502016], [188.30971128894595, -155.22179730803566], [188.06298546519903, -157.2904693810511], [187.8162596414521, -159.3591414540666], [187.56953381770518, -161.4278135270821]], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [[187.56953381770518, -161.4278135270821], [187.4314120542192, -163.50656318883296], [187.29329029073318, -165.5853128505838], [187.1551685272472, -167.66406251233468], [187.01704676376121, -169.74281217408554], [186.87892500027525, -171.82156183583638], [186.74080323678925, -173.90031149758724], [186.60268147330325, -175.97906115933807], [186.46455970981725, -178.05781082108894], [186.32643794633125, -180.1365604828398], [186.18831618284526, -182.21531014459066], [186.0501944193593, -184.29405980634152], [185.9120726558733, -186.37280946809238], [185.7739508923873, -188.45155912984322], [185.63582912890132, -190.53030879159408], [185.49770736541532, -192.60905845334491], [185.35958560192933, -194.68780811509578], [185.22146383844333, -196.76655777684664], [185.08334207495733, -198.8453074385975], [184.94522031147136, -200.92405710034836], [184.80709854798536, -203.00280676209923], [184.66897678449936, -205.0815564238501], [184.5308550210134, -207.16030608560095], [184.3927332575274, -209.23905574735178], [184.2546114940414, -211.31780540910265]], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 25, 'type': 'intersection'}, {'control_points': [[184.2546114940414, -211.31780540910265], [184.37325178198037, -211.5469184857784], [184.49189206991934, -211.7760315624541], [184.61053235785832, -212.00514463912984], [184.7291726457973, -212.23425771580557], [184.84781293373626, -212.4633707924813], [184.96645322167527, -212.69248386915706], [185.0850935096142, -212.9215969458328], [185.2037337975532, -213.1507100225085], [185.32237408549216, -213.37982309918422], [185.44101437343116, -213.60893617585998], [185.5596546613701, -213.83804925253568], [185.6782949493091, -214.0671623292114], [185.79693523724808, -214.29627540588714], [185.91557552518708, -214.52538848256287], [186.03421581312602, -214.75450155923863], [186.152856101065, -214.98361463591434], [186.27149638900397, -215.21272771259007], [186.39013667694297, -215.44184078926577], [186.50877696488192, -215.67095386594153], [186.62741725282092, -215.9000669426173], [186.74605754075986, -216.12918001929302], [186.86469782869887, -216.35829309596872], [186.98333811663784, -216.58740617264445], [187.1019784045768, -216.81651924932018], [187.22061869251576, -217.0456323259959], [187.3392589804548, -217.27474540267167], [187.45789926839376, -217.5038584793474], [187.57653955633273, -217.7329715560231], [187.6951798442717, -217.96208463269886], [187.81382013221068, -218.1911977093746], [187.93246042014965, -218.4203107860503], [188.0511007080886, -218.649423862726], [188.1697409960276, -218.87853693940178], [188.2883812839666, -219.1076500160775], [188.40702157190555, -219.33676309275324], [188.52566185984455, -219.56587616942895], [188.64430214778352, -219.7949892461047], [188.7629424357225, -220.02410232278044], [188.88158272366144, -220.25321539945614], [189.00022301160044, -220.4823284761319], [189.1188632995394, -220.71144155280763], [189.2375035874784, -220.94055462948333], [189.35614387541736, -221.1696677061591], [189.47478416335633, -221.39878078283482], [189.5934244512953, -221.62789385951055], [189.71206473923428, -221.85700693618625], [189.83070502717328, -222.086120012862], [189.94934531511223, -222.31523308953774], [190.0679856030512, -222.54434616621344], [190.18662589099017, -222.7734592428892], [190.30526617892917, -223.00257231956493], [190.42390646686815, -223.23168539624066], [190.54254675480712, -223.4607984729164], [190.6611870427461, -223.68991154959213], [190.77982733068507, -223.91902462626786], [190.89846761862407, -224.1481377029436], [191.017107906563, -224.37725077961932], [191.13574819450199, -224.60636385629505], [191.25438848244096, -224.83547693297078], [191.37302877037996, -225.0645900096465], [191.49166905831893, -225.2937030863222], [191.6103093462579, -225.52281616299797], [191.72894963419688, -225.7519292396737], [191.84758992213585, -225.98104231634943], [191.9662302100748, -226.21015539302513], [192.0848704980138, -226.4392684697009], [192.2035107859528, -226.66838154637662], [192.32215107389175, -226.89749462305235], [192.44079136183075, -227.12660769972808], [192.5594316497697, -227.35572077640379], [192.6780719377087, -227.58483385307954], [192.79671222564767, -227.81394692975528], [192.91535251358664, -228.043060006431], [193.0339928015256, -228.2721730831067], [193.1526330894646, -228.50128615978247], [193.27127337740356, -228.7303992364582], [193.38991366534253, -228.95951231313393], [193.5085539532815, -229.18862538980966], [193.62719424122048, -229.4177384664854], [193.74583452915945, -229.64685154316112], [193.86447481709843, -229.87596461983685], [193.9831151050374, -230.10507769651258], [194.10175539297637, -230.3341907731883], [194.22039568091537, -230.56330384986404], [194.33903596885432, -230.79241692653977], [194.45767625679332, -231.0215300032155], [194.57631654473226, -231.25064307989123], [194.69495683267127, -231.47975615656696], [194.81359712061027, -231.7088692332427], [194.9322374085492, -231.93798230991843], [195.0508776964882, -232.16709538659416], [195.1695179844272, -232.3962084632699], [195.28815827236613, -232.62532153994562], [195.40679856030513, -232.85443461662135], [195.52543884824408, -233.08354769329708], [195.64407913618308, -233.3126607699728], [195.76271942412205, -233.54177384664854], [195.88135971206103, -233.77088692332427], [196.0, -234.0]], 'width': 10, 'left_lanes': 1, 'right_lanes': 1, 'samples': 100, 'type': 'normal'}], 'type': 'urban', 'file_name': 'urban', 'score': 0, 'obstacles': [{'name': 'trafficlightdouble', 'position': (106.77705183931472, -69.5945295956275), 'zRot': 301}, {'name': 'trafficlightdouble', 'position': (120.39194278974706, -61.78513834122688), 'zRot': 208}, {'name': 'trafficlightdouble', 'position': (128.08575104327144, -75.17680854201618), 'zRot': 481}, {'name': 'trafficlightdouble', 'position': (114.75065019732645, -82.97402285662051), 'zRot': 393}, {'name': 'trafficlightdouble', 'position': (179.97459448626378, -166.68299159749267), 'zRot': 338}, {'name': 'trafficlightsingle', 'position': (183.55606804277818, -151.17031907309922), 'zRot': 263}, {'name': 'trafficlightsingle', 'position': (192.08847866405523, -171.85034380891128), 'zRot': 446}], 'success_point': (196, -234), 'ego_lanes': [0, 1, 4, 5, 6, 8, 9], 'participants': [{'id': 'ego', 'init_state': {'position': (1.0, -5.0), 'orientation': 0, 'movementMode': '_BEAMNG', 'speed': 50}, 'waypoints': [{'position': [1.0, 0.0], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [5.232195866085265, -0.00040794985188414423], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [9.152026677687328, -0.003263598815073154], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [12.776175436643756, -0.0110146460008719], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [16.121325144792138, -0.02610879052058523], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [19.20415880397005, -0.05099373148551806], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [22.041359416015066, -0.0881171680069752], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [24.649609982764765, -0.1399267991962615], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [27.04559350605673, -0.20887032416468185], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [29.245992987728524, -0.2973954420235411], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [31.267491429617742, -0.40794985188414445], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [33.12677183356196, -0.5429812528577962], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [34.84051720139875, -0.7049373440558016], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [36.425410534965685, -0.896265824589465], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [37.89813483610035, -1.119414393570092], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [39.27537310664032, -1.3768307501089867], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [40.573808348423185, -1.6709625933174548], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [41.8101235632865, -2.0042576223068007], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [43.001001753067854, -2.379163536188329], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [44.163125919604845, -2.7981280340733456], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [45.31315518893317, -3.2635900548868615], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [46.46268701709474, -3.7761313780597523], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [47.612025605852764, -4.332190214906265], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [48.75994710565162, -4.927644124817882], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [49.90522766693565, -5.558370667186095], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [51.046643440149204, -6.220247401402384], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [52.18297057573662, -6.909151886858243], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [53.31298522414224, -7.620961682945152], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [54.43546353581044, -8.351554349054602], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [55.549181661185536, -9.09680744457808], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [56.6529157507119, -9.85259852890707], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [57.74544195483386, -10.614805161433054], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [58.82553642399577, -11.379304901547533], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [59.89197530864198, -12.141975308641975], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [60.94353475921683, -12.89869394210788], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [61.978990926164684, -13.645338361336728], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [62.997119959929876, -14.377786125720009], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [63.996698010956756, -15.09191479464921], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [64.97650122968967, -15.783601927515813], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [65.93530576657298, -16.448725083711313], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [66.87192556108994, -17.08319892459266], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [67.78664360161144, -17.68438045042473], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [68.68165122297354, -18.25150031072896], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [69.55926729801845, -18.783914374160272], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [70.42181069958849, -19.280978509373572], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [71.27160030052593, -19.74204858502379], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [72.11095497367306, -20.16648046976585], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [72.9421935918722, -20.55363003225466], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [73.76763502796561, -20.902853141145155], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [74.58959815479558, -21.213505665092242], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [75.4104018452044, -21.484943472750842], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [76.2323649720344, -21.71652243277588], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [77.05780640812779, -21.90759841382227], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [77.88904502632694, -22.05752728454494], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [78.72839969947408, -22.165664913598796], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [79.57818930041152, -22.231367169638776], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [80.44073270198155, -22.25398992131979], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [81.31834877702646, -22.23288903729675], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [82.21335639838853, -22.167420386224588], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [83.12807443891006, -22.05693983675822], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [84.06454118781944, -21.901153407351757], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [85.02059657212193, -21.705006394935996], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [85.99084861127, -21.47747730338793], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [86.96982218883046, -21.22764838467318], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [87.9520421883701, -20.964601890757383], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [88.9320334934558, -20.697420073606178], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [89.90432098765434, -20.435185185185187], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [90.8634295545325, -20.186979477460042], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [91.80388407765714, -19.961885202396374], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [92.72020944059513, -19.76898461195982], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [93.60693052691319, -19.617359958116], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [94.45857222017817, -19.51609349283056], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [95.26965940395691, -19.47426746806912], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [96.03471696181624, -19.500964135797318], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [96.74826977732293, -19.605265747980773], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [97.40484273404384, -19.796254556585133], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [97.99896071554575, -20.08301281357602], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [98.52514860539551, -20.474622770919066], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [98.97793128715995, -20.980166680579906], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [99.35183364440583, -21.60872679452416], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [99.6430405301184, -22.36813214277249], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [99.86000500876533, -23.256949661908337], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [100.01667879351278, -24.26959499082241], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [100.12703953454897, -25.400464186812524], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [100.20506488206212, -26.643953307176442], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [100.26473248624049, -27.994458409212015], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [100.32001999727233, -29.446375550216995], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [100.38490506534583, -30.994100787489206], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [100.47336534064928, -32.632030178326474], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [100.59937847337093, -34.35455978002656], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [100.77692211369899, -36.15608564988729], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [101.01997391182168, -38.031003845206484], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [101.34251151792729, -39.97371042328189], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [101.75851258220405, -41.97860144141137], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [102.28195475484017, -44.04007295689266], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [102.92681568602393, -46.15252102702361], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [103.70707302594354, -48.31034170910204], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [104.63670442478727, -50.5079310604257], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [105.72968753274336, -52.73968513829244], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [107.0, -55.0], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [107.0, -55.0], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [107.42874646285628, -55.71457743809379], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [107.85749292571253, -56.42915487618757], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [108.28623938856882, -57.14373231428136], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [108.7149858514251, -57.85830975237515], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [109.14373231428137, -58.57288719046893], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [109.57247877713763, -59.28746462856272], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [110.00122523999391, -60.00204206665651], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [110.42997170285018, -60.7166195047503], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [110.85871816570645, -61.43119694284408], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [111.28746462856273, -62.145774380937866], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [111.716211091419, -62.860351819031656], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [112.14495755427527, -63.57492925712544], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [112.57370401713155, -64.28950669521922], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [113.00245047998781, -65.00408413331301], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [113.43119694284408, -65.7186615714068], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [113.85994340570035, -66.4332390095006], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [114.28868986855663, -67.14781644759438], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [114.7174363314129, -67.86239388568816], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [115.14618279426917, -68.57697132378195], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [115.57492925712545, -69.29154876187573], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [116.00367571998171, -70.00612619996951], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [116.43242218283798, -70.7207036380633], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [116.86116864569426, -71.4352810761571], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [117.28991510855053, -72.14985851425088], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [117.28991510855053, -72.14985851425088], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [117.93303480283495, -73.22172467139156], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [118.57615449711933, -74.29359082853223], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [119.21927419140376, -75.36545698567292], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [119.86239388568816, -76.4373231428136], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [120.50551357997257, -77.50918929995429], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [121.14863327425698, -78.58105545709496], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [121.7917529685414, -79.65292161423564], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [122.4348726628258, -80.72478777137633], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [123.0779923571102, -81.796653928517], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [123.72111205139461, -82.86852008565768], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [124.36423174567904, -83.94038624279835], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [125.00735143996343, -85.01225239993903], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [125.65047113424782, -86.08411855707972], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [126.29359082853225, -87.1559847142204], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [126.93671052281665, -88.22785087136108], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [127.57983021710106, -89.29971702850175], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [128.22294991138548, -90.37158318564244], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [128.86606960566988, -91.44344934278311], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [129.50918929995427, -92.5153154999238], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [130.1523089942387, -93.58718165706448], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [130.7954286885231, -94.65904781420515], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [131.4385483828075, -95.73091397134583], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [132.08166807709193, -96.8027801284865], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [132.72478777137633, -97.8746462856272], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [132.72478777137633, -97.8746462856272], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [133.03638306554706, -98.57657493912912], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [133.34844231485894, -99.28072176663298], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [133.66098577211397, -99.98693295176092], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [133.97403369011386, -100.6950546781349], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [134.28760632166046, -101.40493312937701], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [134.6017239195557, -102.11641448910929], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [134.9164067366013, -102.82934494095373], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [135.2316750255992, -103.54357066853245], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [135.54754903935108, -104.25893785546744], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [135.864049030659, -104.97529268538082], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [136.18119525232464, -105.69248134189455], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [136.49900795714993, -106.41035000863067], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [136.81750739793668, -107.12874486921135], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [137.1367138274867, -107.84751210725848], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [137.45664749860182, -108.5664979063942], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [137.77732866408388, -109.2855484502405], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [138.09877757673482, -110.00450992241949], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [138.42101448935637, -110.72322850655311], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [138.74405965475043, -111.44155038626354], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [139.0679333257188, -112.15932174517275], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [139.39265575506332, -112.87638876690272], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [139.71824719558586, -113.59259763507563], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [140.0447279000882, -114.3077945333134], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [140.37211812137227, -115.0218256452382], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [140.7004381122398, -115.73453715447194], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [141.0297081254928, -116.4457752446368], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [141.35994841393293, -117.15538609935473], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [141.69117923036205, -117.86321590224779], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [142.0234208275821, -118.56911083693802], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [142.35669345839486, -119.2729170870475], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [142.69101737560214, -119.97448083619825], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [143.02641283200583, -120.67364826801231], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [143.3629000804078, -121.37026556611177], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [143.70049937360983, -122.06417891411863], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [144.03923096441372, -122.7552344956549], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [144.37911510562142, -123.44327849434269], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [144.72017205003468, -124.12815709380405], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [145.06242205045533, -124.80971647766094], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [145.40588535968524, -125.4878028295355], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [145.75058223052636, -126.16226233304972], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [146.09653291578041, -126.83294117182571], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [146.44375766824916, -127.49968552948538], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [146.7922767407346, -128.1623415896509], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [147.1421103860385, -128.82075553594433], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [147.49327885696266, -129.4747735519876], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [147.84580240630893, -130.12424182140276], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [148.19970128687928, -130.76900652781202], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [148.55499575147542, -131.40891385483727], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [148.91170605289918, -132.04380998610054], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [149.26985244395252, -132.67354110522402], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [149.62945517743714, -133.29795339582964], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [149.99053450615492, -133.91689304153945], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [150.35311068290773, -134.53020622597552], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [150.71720396049741, -135.1377391327599], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [151.08283459172577, -135.7393379455146], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [151.4500228293947, -136.3348488478617], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [151.81878892630596, -136.92411802342326], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [152.18915313526142, -137.50699165582125], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [152.56113570906297, -138.0833159286778], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [152.9347569005124, -138.65293702561493], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [153.31003696241157, -139.21570113025464], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [153.6869961475623, -139.771454426219], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [154.06565470876643, -140.32004309713008], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [154.4460328988258, -140.8613133266099], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [154.82815097054225, -141.39511129828054], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [155.21202917671764, -141.92128319576398], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [155.5976877701538, -142.4396752026823], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [155.98514700365254, -142.95013350265756], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [156.37442713001576, -143.45250427931177], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [156.76554840204523, -143.946633716267], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [157.15853107254284, -144.4323679971453], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [157.5533953943104, -144.9095533055687], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [157.9501616201498, -145.37803582515926], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [158.34885000286278, -145.837661739539], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [158.74948079525126, -146.28827723232996], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [159.15207425011707, -146.7297284871542], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [159.55665062026202, -147.16186168763377], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [159.96323015848796, -147.58452301739072], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [160.37183311759674, -147.99755866004705], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [160.78247975039022, -148.40081479922492], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [161.19519030967018, -148.79413761854624], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [161.6099850482385, -149.1773733016331], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [162.026884218897, -149.55036803210757], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [162.44590807444757, -149.91296799359165], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [162.867076867692, -150.26501936970743], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [163.2904108514321, -150.60636834407694], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [163.71593027846978, -150.93686110032223], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [164.14365540160682, -151.25634382206533], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [164.57360647364513, -151.5646626929283], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [165.00580374738647, -151.86166389653314], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [165.4402674756327, -152.14719361650197], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [165.8770179111857, -152.42109803645678], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [166.31607530684727, -152.68322334001962], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [166.7574599154193, -152.93341571081254], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [167.20119198970357, -153.1715213324576], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [167.64729178250192, -153.39738638857685], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [168.09577954661623, -153.61085706279226], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [168.5466755348483, -153.81177953872597], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [169.0, -154.0], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [169.0, -154.0], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [169.77373057573772, -154.3094922302951], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [170.54746115147543, -154.61898446059016], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [171.32119172721315, -154.92847669088525], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [172.09492230295086, -155.23796892118037], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [172.86865287868858, -155.54746115147543], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [173.6423834544263, -155.85695338177052], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [174.416114030164, -156.16644561206562], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [175.18984460590173, -156.4759378423607], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [175.96357518163944, -156.7854300726558], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [176.73730575737716, -157.09492230295086], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [177.51103633311487, -157.40441453324595], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [178.2847669088526, -157.71390676354105], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [179.0584974845903, -158.02339899383614], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [179.83222806032802, -158.33289122413123], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [180.60595863606574, -158.6423834544263], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [181.37968921180345, -158.9518756847214], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [182.15341978754117, -159.26136791501648], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [182.92715036327888, -159.57086014531157], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [183.7008809390166, -159.88035237560666], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.47461151475432, -160.18984460590175], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.24834209049203, -160.49933683619685], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.02207266622975, -160.8088290664919], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.79580324196746, -161.118321296787], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.56953381770518, -161.4278135270821], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.56953381770518, -161.4278135270821], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.4314120542192, -163.50656318883296], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.29329029073318, -165.5853128505838], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.1551685272472, -167.66406251233468], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.01704676376121, -169.74281217408554], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.87892500027525, -171.82156183583638], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.74080323678925, -173.90031149758724], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.60268147330325, -175.97906115933807], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.46455970981725, -178.05781082108894], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.32643794633125, -180.1365604828398], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.18831618284526, -182.21531014459066], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.0501944193593, -184.29405980634152], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.9120726558733, -186.37280946809238], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.7739508923873, -188.45155912984322], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.63582912890132, -190.53030879159408], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.49770736541532, -192.60905845334491], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.35958560192933, -194.68780811509578], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.22146383844333, -196.76655777684664], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.08334207495733, -198.8453074385975], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.94522031147136, -200.92405710034836], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.80709854798536, -203.00280676209923], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.66897678449936, -205.0815564238501], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.5308550210134, -207.16030608560095], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.3927332575274, -209.23905574735178], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.2546114940414, -211.31780540910265], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.2546114940414, -211.31780540910265], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.37325178198037, -211.5469184857784], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.49189206991934, -211.7760315624541], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.61053235785832, -212.00514463912984], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.7291726457973, -212.23425771580557], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.84781293373626, -212.4633707924813], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [184.96645322167527, -212.69248386915706], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.0850935096142, -212.9215969458328], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.2037337975532, -213.1507100225085], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.32237408549216, -213.37982309918422], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.44101437343116, -213.60893617585998], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.5596546613701, -213.83804925253568], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.6782949493091, -214.0671623292114], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.79693523724808, -214.29627540588714], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [185.91557552518708, -214.52538848256287], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.03421581312602, -214.75450155923863], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.152856101065, -214.98361463591434], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.27149638900397, -215.21272771259007], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.39013667694297, -215.44184078926577], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.50877696488192, -215.67095386594153], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.62741725282092, -215.9000669426173], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.74605754075986, -216.12918001929302], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.86469782869887, -216.35829309596872], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [186.98333811663784, -216.58740617264445], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.1019784045768, -216.81651924932018], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.22061869251576, -217.0456323259959], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.3392589804548, -217.27474540267167], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.45789926839376, -217.5038584793474], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.57653955633273, -217.7329715560231], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.6951798442717, -217.96208463269886], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.81382013221068, -218.1911977093746], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [187.93246042014965, -218.4203107860503], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [188.0511007080886, -218.649423862726], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [188.1697409960276, -218.87853693940178], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [188.2883812839666, -219.1076500160775], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [188.40702157190555, -219.33676309275324], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [188.52566185984455, -219.56587616942895], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [188.64430214778352, -219.7949892461047], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [188.7629424357225, -220.02410232278044], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [188.88158272366144, -220.25321539945614], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [189.00022301160044, -220.4823284761319], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [189.1188632995394, -220.71144155280763], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [189.2375035874784, -220.94055462948333], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [189.35614387541736, -221.1696677061591], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [189.47478416335633, -221.39878078283482], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [189.5934244512953, -221.62789385951055], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [189.71206473923428, -221.85700693618625], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [189.83070502717328, -222.086120012862], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [189.94934531511223, -222.31523308953774], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [190.0679856030512, -222.54434616621344], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [190.18662589099017, -222.7734592428892], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [190.30526617892917, -223.00257231956493], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [190.42390646686815, -223.23168539624066], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [190.54254675480712, -223.4607984729164], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [190.6611870427461, -223.68991154959213], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [190.77982733068507, -223.91902462626786], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [190.89846761862407, -224.1481377029436], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [191.017107906563, -224.37725077961932], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [191.13574819450199, -224.60636385629505], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [191.25438848244096, -224.83547693297078], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [191.37302877037996, -225.0645900096465], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [191.49166905831893, -225.2937030863222], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [191.6103093462579, -225.52281616299797], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [191.72894963419688, -225.7519292396737], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [191.84758992213585, -225.98104231634943], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [191.9662302100748, -226.21015539302513], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [192.0848704980138, -226.4392684697009], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [192.2035107859528, -226.66838154637662], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [192.32215107389175, -226.89749462305235], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [192.44079136183075, -227.12660769972808], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [192.5594316497697, -227.35572077640379], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [192.6780719377087, -227.58483385307954], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [192.79671222564767, -227.81394692975528], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [192.91535251358664, -228.043060006431], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [193.0339928015256, -228.2721730831067], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [193.1526330894646, -228.50128615978247], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [193.27127337740356, -228.7303992364582], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [193.38991366534253, -228.95951231313393], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [193.5085539532815, -229.18862538980966], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [193.62719424122048, -229.4177384664854], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [193.74583452915945, -229.64685154316112], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [193.86447481709843, -229.87596461983685], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [193.9831151050374, -230.10507769651258], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [194.10175539297637, -230.3341907731883], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [194.22039568091537, -230.56330384986404], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [194.33903596885432, -230.79241692653977], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [194.45767625679332, -231.0215300032155], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [194.57631654473226, -231.25064307989123], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [194.69495683267127, -231.47975615656696], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [194.81359712061027, -231.7088692332427], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [194.9322374085492, -231.93798230991843], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [195.0508776964882, -232.16709538659416], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [195.1695179844272, -232.3962084632699], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [195.28815827236613, -232.62532153994562], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [195.40679856030513, -232.85443461662135], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [195.52543884824408, -233.08354769329708], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [195.64407913618308, -233.3126607699728], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [195.76271942412205, -233.54177384664854], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}, {'position': [195.88135971206103, -233.77088692332427], 'tolerance': 2.5, 'movementMode': '_BEAMNG'}], 'model': 'ETK800'}]}
        temp_list = [individual]
        temp_list = self._spline_population(temp_list)
        temp_list = _merge_lanes(temp_list)
        build_all_xml(temp_list)
        self.population_list = []

    def get_test(self):
        """Returns the two first test files starting with "files_name".
        :return: Tuple of the path to the dbe and dbc file.
        """
        destination_path = path.dirname(path.realpath(__file__)) + "\\scenario"
        xml_names = destination_path + "\\" + self.files_name + "*"
        matches = glob(xml_names)
        iterator = 0
        self.genetic_algorithm()
        while iterator < self.POPULATION_SIZE * 2 - 1:
            yield Path(matches[iterator + 1]), Path(matches[iterator])
            iterator += 2

# TODO Desired features:
#       TODO Lane switch when turning for multiple lanes
#       TODO Make cars stop in front of intersection
#       TODO Waypoints are broken
#       TODO Add other participants
#       TODO Create init population
#       TODO Mutation
#       TODO Repair function
#       TODO Mutation validity checks
#       TODO Crossover
#       TODO Refactor
#       TODO Comments
#       TODO Fix lane markings

# TODO Verifier:

# TODO May-have:
#       TODO Remove redundant XML information
#       TODO Daytime
#       TODO Add weather presets
#       TODO Double test cases by placing spawn point on the other side
#       TODO Improve performance
#       TODO Make all objects colidable
#       TODO Fix Shapely's TopologyException
#       TODO Converter:
#           TODO Add input checking
#           TODO Implement Sensor deployment
#       TODO Control traffic lights (not possible)
