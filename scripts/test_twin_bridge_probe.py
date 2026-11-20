"""Classification of bridge replies in twin_bridge_probe.py (standard library only).

  python3 -m unittest discover -s scripts -p 'test_twin_bridge_probe.py'
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import twin_bridge_probe as probe  # noqa: E402


class RefusedTest(unittest.TestCase):
    def test_no_reply_is_a_refusal(self):
        self.assertTrue(probe._refused([])[0])

    def test_rosapi_glob_refusal_is_a_refusal(self):
        replies = [{'op': 'service_response', 'result': True, 'values': {
            'successful': False, 'reason': 'Parameter x does not match any of the glob strings'}}]
        ok, detail = probe._refused(replies)
        self.assertTrue(ok)
        self.assertIn('glob', detail)

    def test_an_answered_service_is_not_a_refusal(self):
        replies = [{'op': 'service_response', 'result': True, 'values': {'successful': True}}]
        self.assertFalse(probe._refused(replies)[0])

    def test_data_coming_back_is_not_a_refusal(self):
        self.assertFalse(probe._refused([{'op': 'publish', 'topic': '/cmd_vel_nav'}])[0])

    def test_answered(self):
        self.assertTrue(probe._answered([{'op': 'service_response', 'result': True, 'values': {}}])[0])
        self.assertFalse(probe._answered([])[0])


if __name__ == '__main__':
    unittest.main()
