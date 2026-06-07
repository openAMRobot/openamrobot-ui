import unittest
import tempfile


class PackageSmokeTest(unittest.TestCase):
    def test_static_modules_import(self):
        import openamr_ui_package.flask_app  # noqa: F401
        import openamr_ui_package.map_relay  # noqa: F401
        import openamr_ui_package.nav_relays  # noqa: F401


class FlaskApiTest(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.TemporaryDirectory()

        import openamr_ui_package.flask_app as flask_app

        self.flask_app = flask_app
        flask_app.BLOCK_PROGRAMS_DIR = f"{self.tmpdir.name}/block_programs"
        flask_app.BLOCK_LOCATIONS_FILE = f"{self.tmpdir.name}/block_locations.json"
        flask_app.BLOCK_RUN_HISTORY_FILE = (
            f"{self.tmpdir.name}/block_run_history.json"
        )
        flask_app.app.config.update(TESTING=True)
        self.client = flask_app.app.test_client()

    def tearDown(self):
        self.tmpdir.cleanup()

    def test_block_program_api_round_trip(self):
        response = self.client.post(
            "/api/block-programs/Test Program",
            json={"workspace": {"blocks": []}, "plan": [{"type": "wait"}]},
        )
        self.assertEqual(response.status_code, 201)

        response = self.client.get("/api/block-programs")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["programs"][0]["name"], "Test Program")

        response = self.client.get("/api/block-programs/Test Program")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["workspace"], {"blocks": []})

        response = self.client.delete("/api/block-programs/Test Program")
        self.assertEqual(response.status_code, 200)

    def test_block_location_api_round_trip(self):
        response = self.client.post(
            "/api/block-locations/Dock",
            json={"x": 1.2, "y": 3.4, "yaw": 1.57},
        )
        self.assertEqual(response.status_code, 201)

        response = self.client.get("/api/block-locations")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["locations"]["Dock"]["x"], 1.2)

        response = self.client.delete("/api/block-locations/Dock")
        self.assertEqual(response.status_code, 200)

    def test_block_run_history_api_round_trip(self):
        response = self.client.post(
            "/api/block-run-history",
            json={
                "program_name": "Safe Motion Test",
                "status": "success",
                "started_at": "2026-01-01T00:00:00+00:00",
                "duration_ms": 1000,
                "steps_total": 1,
                "steps_completed": 1,
            },
        )
        self.assertEqual(response.status_code, 201)

        response = self.client.get("/api/block-run-history")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["history"][0]["status"], "success")

        response = self.client.delete("/api/block-run-history")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["history"], [])


if __name__ == "__main__":
    unittest.main()
