import unittest

from stage1_sim.course import CourseMap
from stage1_sim.grid import LETTERS, validate_run_inputs
from stage1_sim.session import run_stage1


class GridInputTest(unittest.TestCase):
    def test_requires_four_letters_and_keeps_repeats(self):
        start, bearing, goals = validate_run_inputs("s", "ne", ["H", "c", "C", "x"])
        self.assertEqual((start, bearing, goals), ("S", "NE", ["H", "C", "C", "X"]))

    def test_rejects_bad_input(self):
        with self.assertRaises(ValueError):
            validate_run_inputs("S", "E", ["T", "N", "H"])
        with self.assertRaises(ValueError):
            validate_run_inputs("Z", "E", ["T", "N", "H", "A"])
        with self.assertRaises(ValueError):
            validate_run_inputs("S", "EE", ["T", "N", "H", "A"])


class CourseTest(unittest.TestCase):
    def test_every_centre_is_its_own_cell(self):
        course = CourseMap()
        for letter in LETTERS:
            x, y = course.cells[letter].center
            self.assertEqual(course.letter_at(x, y), letter)
            self.assertFalse(course.occupancy[course.world_to_grid(x, y)[1], course.world_to_grid(x, y)[0]])

    def test_north_row_is_above_south_row(self):
        course = CourseMap()
        self.assertGreater(course.cells["A"].center[1], course.cells["S"].center[1])
        self.assertGreater(course.cells["X"].center[0], course.cells["S"].center[0])


class Stage1Test(unittest.TestCase):
    def test_spawn_goals_and_first_path(self):
        result = run_stage1("S", "E", ["T", "N", "H", "A"])
        self.assertTrue(result.spawn_ok)
        self.assertTrue(result.goals_ok)
        self.assertEqual(result.first_goal, "T")
        self.assertGreater(len(result.path), 1)
        self.assertTrue(result.plan_ok)
        self.assertTrue(result.ok)

    def test_same_cell_is_a_valid_plan(self):
        result = run_stage1("H", "N", ["H", "H", "A", "X"])
        self.assertTrue(result.ok)
        self.assertGreaterEqual(len(result.path), 1)


if __name__ == "__main__":
    unittest.main()
