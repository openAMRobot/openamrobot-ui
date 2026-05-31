# Static Frontend Assets

This folder receives the compiled React production build served by Flask.

Do not edit generated files here directly. Change frontend source under the
repository-level `web/` directory, then run:

```bash
bash scripts/build_frontend.sh
bash scripts/sync_frontend_to_ros.sh
```
