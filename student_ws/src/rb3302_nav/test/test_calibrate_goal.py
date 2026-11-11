"""Calibration persistence and input checks; these tests do not command motion."""
from contextlib import redirect_stdout
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image
import yaml

from rb3302_nav.calibrate_goal import main, save_centre


class CalibrationTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        Image.fromarray(np.full((10, 10), 254, dtype=np.uint8)).save(self.root / 'map.pgm')
        (self.root / 'map.yaml').write_text(yaml.safe_dump(dict(
            image='map.pgm', resolution=1.0, origin=[0, 0, 0],
            free_thresh=.196, occupied_thresh=.65)))
        self.path = self.root / 'course.yaml'
        self.original = ('# Keep this course comment\nmap_yaml: map.yaml\nframe_id: map\n'
                         'calibrated: false\nheading_offset_deg: 12.0\nletters:\n'
                         '  F:\n    x: 2.60 # original estimate\n    y: 0.2625\n'
                         '  S:\n    x: 0.5\n    y: 0.5\n')
        self.path.write_text(self.original)

    def test_update_preserves_other_letters_settings_comments_and_backup(self):
        previous, backup = save_centre(self.path, 'F', 2.396, .145)
        self.assertEqual(previous, (2.6, .2625))
        self.assertEqual(backup.read_text(), self.original)
        self.assertEqual(self.path.read_text(), self.original.replace(
            'x: 2.60', 'x: 2.396000').replace('y: 0.2625', 'y: 0.145000'))
        data = yaml.safe_load(self.path.read_text())
        self.assertFalse(data['calibrated'])
        self.assertEqual(data['letters']['S'], {'x': .5, 'y': .5})
        self.assertEqual(data['heading_offset_deg'], 12.0)

    def test_rejects_invalid_measurements_without_modifying_course(self):
        for point in [(float('nan'), .5), (.5, float('inf')), (-.1, .5), (10, .5)]:
            with self.subTest(point=point), self.assertRaises(ValueError):
                save_centre(self.path, 'F', *point)
            self.assertEqual(self.path.read_text(), self.original)
        self.assertFalse(self.path.with_name('course.yaml.bak').exists())

    def test_rejects_occupied_and_unknown_centres(self):
        for pixel in (0, 205):
            Image.fromarray(np.full((10, 10), pixel, dtype=np.uint8)).save(self.root / 'map.pgm')
            with self.subTest(pixel=pixel), self.assertRaisesRegex(ValueError, 'occupied or unknown'):
                save_centre(self.path, 'F', 2.396, .145)
            self.assertEqual(self.path.read_text(), self.original)

    def test_missing_letter_does_not_create_an_unmeasured_goal(self):
        with self.assertRaisesRegex(ValueError, 'Missing letter'):
            save_centre(self.path, 'X', 2.396, .145)
        self.assertEqual(self.path.read_text(), self.original)

    def test_failed_replace_retains_original_and_removes_temporary_file(self):
        with patch('rb3302_nav.calibrate_goal.os.replace', side_effect=OSError('write failed')):
            with self.assertRaises(OSError):
                save_centre(self.path, 'F', 2.396, .145)
        self.assertEqual(self.path.read_text(), self.original)
        self.assertEqual({p.name for p in self.root.iterdir()},
                         {'map.yaml', 'map.pgm', 'course.yaml', 'course.yaml.bak'})

    def test_cli_accepts_recorded_coordinates_without_live_ros(self):
        output = io.StringIO()
        with redirect_stdout(output), patch('rb3302_nav.calibrate_goal.capture_position') as capture:
            main(['--course', str(self.path), '--letter', 'f', '--x', '2.396', '--y', '.145'])
        capture.assert_not_called()
        self.assertIn('Saved F:', output.getvalue())
        self.assertEqual(yaml.safe_load(self.path.read_text())['letters']['F'],
                         {'x': 2.396, 'y': .145})


if __name__ == '__main__':
    unittest.main()
