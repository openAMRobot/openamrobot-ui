"""/api/limits and /api/voice-plan enforce the plan limits on the server.

Imports flask_app (rclpy, Flask), so it runs under colcon test. The model
call is replaced by a stub that returns the plan under test, so each plan is
posted through the real endpoint. The pure rule test is test_voice_limits.py.
"""

import unittest
from unittest import mock

NAN = float('nan')
INF = float('inf')

REJECTED = {
    'linear speed above limit': [{'type': 'set_speed', 'linear': 0.25, 'angular': 0}],
    'angular speed above limit': [{'type': 'rotate_for', 'angular': 2.5, 'seconds': 1}],
    'drive distance above limit': [{'type': 'drive_for', 'linear': 0.2, 'seconds': 26}],
    'duration above limit': [{'type': 'wait', 'seconds': 300}],
    'nested above limit': [{'type': 'repeat', 'times': 2, 'actions': [
        {'type': 'drive_for', 'linear': 1, 'seconds': 1}]}],
    'NaN speed': [{'type': 'set_speed', 'linear': NAN, 'angular': 0}],
    'infinite duration': [{'type': 'wait', 'seconds': INF}],
    'string speed': [{'type': 'drive_for', 'linear': 'fast', 'seconds': 1}],
    'missing field': [{'type': 'rotate_for', 'angular': 1}],
}


class VoicePlanLimitsApiTest(unittest.TestCase):

    def setUp(self):
        import openamr_ui_package.flask_app as flask_app

        self.app = flask_app
        flask_app.app.config.update(TESTING=True)
        self.client = flask_app.app.test_client()

    def _post(self, plan):
        with mock.patch.object(self.app, 'generate_voice_plan_actions', return_value=plan):
            return self.client.post('/api/voice-plan', json={'transcript': 'go'})

    def test_limits_endpoint_serves_the_config(self):
        response = self.client.get('/api/limits')
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()['limits'], {
            'max_linear_speed_mps': 0.2,
            'max_angular_speed_radps': 2.0,
            'max_distance_per_action_m': 5.0,
            'max_duration_per_action_s': 30.0,
        })

    def test_out_of_range_and_malformed_plans_are_rejected_per_action(self):
        for name, plan in REJECTED.items():
            with self.subTest(name):
                response = self._post([{'type': 'wait', 'seconds': 1}, *plan])
                self.assertEqual(response.status_code, 422)
                body = response.get_json()
                self.assertNotIn('plan', body)
                (error,) = body['errors']
                self.assertTrue(error['path'].startswith('1'))
                self.assertTrue(error['reasons'])
                self.assertIn(error['reasons'][0], body['message'])

    def test_plan_within_limits_is_returned_unchanged(self):
        plan = [{'type': 'set_speed', 'linear': 0.2, 'angular': 2.0},
                {'type': 'drive_for', 'linear': 0.1, 'seconds': 30}]
        response = self._post(plan)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()['plan'], plan)

    def test_model_prompt_states_the_backend_limits(self):
        limits = self.app.load_plan_limits()
        prompt = self.app.build_voice_plan_system_prompt({}, limits)
        for value in ('0.2 m/s', '2.0 rad/s', '30.0 s', '5.0 m'):
            self.assertIn(value, prompt)

    def test_invalid_limits_file_fails_closed(self):
        with mock.patch.object(self.app, 'PLAN_LIMITS_FILE', '/nonexistent/voice_limits.yaml'):
            self.assertEqual(self.client.get('/api/limits').status_code, 500)
            response = self._post([{'type': 'wait', 'seconds': 1}])
            self.assertEqual(response.status_code, 500)


if __name__ == '__main__':
    unittest.main()
