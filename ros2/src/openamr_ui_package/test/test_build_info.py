"""The app serves index.html only for a bundle that matches build-info.json.

Review of openamrobot-ui#50, point 5: a colcon-only install served a stale
index.html whose main JS was not in the tree. Needs the ROS overlay (imports
flask_app, like test_smoke.py).
"""
import hashlib
import json
import os
import tempfile
import unittest


def _write_bundle(root, js=b"console.log('ui');", info_js=None, info_sha=None):
    os.makedirs(os.path.join(root, "static", "js"), exist_ok=True)
    main_js = "/static/js/main.abc123.js"
    with open(os.path.join(root, "static", "js", "main.abc123.js"), "wb") as f:
        f.write(js)
    with open(os.path.join(root, "index.html"), "w") as f:
        f.write("<!doctype html><div id=root></div>")
    with open(os.path.join(root, "asset-manifest.json"), "w") as f:
        json.dump({"files": {"main.js": main_js}}, f)
    info = {
        "commit": "0" * 40,
        "main_js": info_js or main_js,
        "main_js_sha256": info_sha or hashlib.sha256(js).hexdigest(),
    }
    with open(os.path.join(root, "build-info.json"), "w") as f:
        json.dump(info, f)


class BuildInfoTest(unittest.TestCase):
    def setUp(self):
        import openamr_ui_package.flask_app as flask_app

        self.flask_app = flask_app
        self.tmp = tempfile.TemporaryDirectory()
        self.saved = flask_app.REACT_BUILD_DIR
        flask_app.REACT_BUILD_DIR = self.tmp.name
        flask_app._bundle_cache.update(key=None, status=None)
        flask_app.app.config.update(TESTING=True)
        self.client = flask_app.app.test_client()

    def tearDown(self):
        self.flask_app.REACT_BUILD_DIR = self.saved
        self.tmp.cleanup()

    def test_no_build_info_refuses_index(self):
        with open(os.path.join(self.tmp.name, "index.html"), "w") as f:
            f.write("<!doctype html>stale")
        response = self.client.get("/")
        self.assertEqual(response.status_code, 503)
        self.assertIn(b"No verified frontend build", response.data)
        self.assertNotIn(b"stale", response.data)
        self.assertEqual(self.client.get("/map").status_code, 503)
        self.assertEqual(self.client.get("/index.html").status_code, 503)
        self.assertEqual(self.client.get("/api/build-info").status_code, 503)

    def test_matching_bundle_is_served_and_reported(self):
        _write_bundle(self.tmp.name)
        self.assertEqual(self.client.get("/").status_code, 200)
        response = self.client.get("/api/build-info")
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.get_json()["ok"])
        self.assertEqual(response.get_json()["commit"], "0" * 40)

    def test_changed_main_js_is_refused(self):
        _write_bundle(self.tmp.name, info_sha="f" * 64)
        self.assertEqual(self.client.get("/").status_code, 503)
        self.assertIn("sha256", self.client.get("/api/build-info").get_json()["reason"])

    def test_manifest_naming_another_main_js_is_refused(self):
        _write_bundle(self.tmp.name, info_js="/static/js/main.old.js")
        self.assertEqual(self.client.get("/").status_code, 503)


if __name__ == "__main__":
    unittest.main()
