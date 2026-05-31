import unittest


class PackageSmokeTest(unittest.TestCase):
    def test_static_modules_import(self):
        import openamr_ui_package.flask_app  # noqa: F401
        import openamr_ui_package.map_relay  # noqa: F401
        import openamr_ui_package.nav_relays  # noqa: F401


if __name__ == "__main__":
    unittest.main()
