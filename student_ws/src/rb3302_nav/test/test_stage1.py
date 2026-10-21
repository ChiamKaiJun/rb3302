"""Input, map-coordinate and output regressions; these do not run Nav2."""
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image
import yaml
from rb3302_nav.course import Course, CourseMap, Run
from rb3302_nav.render import render_stage1


class Stage1Test(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        # Explicit test fixture only: not another shipped demo arena.
        Image.fromarray(np.full((20, 20), 254, dtype=np.uint8)).save(self.root/'map.pgm')
        self.meta = dict(image='map.pgm', mode='trinary', resolution=.05,
                         origin=[0., 0., 0.], negate=0, free_thresh=.196, occupied_thresh=.65)
        self.write_map()
        self.course_data = dict(map_yaml='map.yaml', frame_id='map', calibrated=False,
                               letters={'S': {'x': .3, 'y': .3}, 'T': {'x': .6, 'y': .3}})
        self.write_course()

    def write_map(self):
        (self.root/'map.yaml').write_text(yaml.safe_dump(self.meta))
        return CourseMap(self.root/'map.yaml')

    def write_course(self):
        (self.root/'course.yaml').write_text(yaml.safe_dump(self.course_data))
        return Course(self.root/'course.yaml')

    def run_file(self, **overrides):
        data = dict(start='s', bearing='ne', goals=['T', 'T', 'S', 'T'])
        data.update(overrides)
        path = self.root/'run.yaml'
        path.write_text(yaml.safe_dump(data))
        return path

    def test_order_repeats_and_case(self):
        run = Run.load(self.run_file())
        self.assertEqual((run.start, run.bearing, run.goals), ('S', 'NE', ('T', 'T', 'S', 'T')))
        self.write_course().validate(run)

    def test_invalid_inputs(self):
        for key, value in [('start', ''), ('start', 'AB'), ('start', 'Z'),
                           ('bearing', 'east'), ('goals', ['A']*3), ('goals', ['A']*5),
                           ('goals', ['A', 'B', '', 'D']), ('goals', 'ABCD')]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                Run.load(self.run_file(**{key: value}))

    def test_all_eight_bearings_and_offset(self):
        self.course_data['heading_offset_deg'] = 30
        course = self.write_course()
        for bearing, degrees in [('E',0),('NE',45),('N',90),('NW',135),('W',180),
                                 ('SW',-135),('S',-90),('SE',-45)]:
            run = Run.load(self.run_file(bearing=bearing))
            self.assertAlmostEqual(course.start_pose(run)[2], math.radians(degrees+30))

    def test_gray_walls_unknown_and_white(self):
        Image.fromarray(np.array([[0, 50, 88, 205, 254, 255]], dtype=np.uint8)).save(self.root/'map.pgm')
        course = self.write_map()
        self.assertEqual(course.occupied.tolist(), [[True, True, True, False, False, False]])
        self.assertEqual(course.occupancy.tolist(), [[True, True, True, True, False, False]])

    def test_negated_map(self):
        self.meta['negate'] = 1
        self.assertTrue(self.write_map().occupancy.all())

    def test_image_orientation_and_rotated_origin(self):
        image = np.full((20, 20), 254, dtype=np.uint8)
        image[0, 0] = 0
        Image.fromarray(image).save(self.root/'map.pgm')
        self.meta['origin'] = [-2., 3., math.pi/2]
        course = self.write_map()
        self.assertTrue(course.occupancy[19, 0])
        for i, j in [(0, 0), (19, 19), (4, 12)]:
            self.assertEqual(course.world_to_grid(*course.grid_to_world(i, j)), (i, j))

    def test_out_of_bounds_not_clipped(self):
        course = self.write_map()
        for point in [(-.1, .5), (1., .5), (.5, 1.)]:
            with self.assertRaises(ValueError):
                course.world_to_grid(*point)

    def test_missing_letter_and_blocked_goal(self):
        course = self.write_course()
        with self.assertRaisesRegex(ValueError, 'Missing'):
            course.validate(Run.load(self.run_file(goals=['X']*4)))
        course.map.occupancy[:] = True
        with self.assertRaisesRegex(ValueError, 'occupied'):
            course.validate(Run.load(self.run_file()))

    def test_draws_path_with_repeated_goals(self):
        course = self.write_course()
        output = self.root/'result.png'
        render_stage1(course.map, (.3,.3), 0., [('T',(.6,.3)),('T',(.6,.3)),('S',(.3,.3)),('T',(.6,.3))],
                      np.array([[.3,.3],[.4,.3],[.5,.3],[.6,.3]]), str(output))
        with Image.open(output) as image:
            self.assertGreater(image.width, 0)
            self.assertGreater(len(image.getcolors(image.width*image.height)), 3)


if __name__ == '__main__':
    unittest.main()
