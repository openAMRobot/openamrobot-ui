# Templates

The `index.html` in this folder is a stale leftover from before the current
build pipeline existed — it references a `static/js/`/`static/css/` layout
that no longer matches how the app is actually built and served.
`flask_app.py` doesn't call Flask's `render_template()` or read from this
folder at all; it serves the React production build directly from
`static/app/` via `serve_spa()` (see
[Lesson 05](../../../../../docs/lessons/05-backend-nodes-in-detail.md)).

Normal frontend changes go in the repository-level `web/` directory, not
here. This folder is otherwise unused — safe to ignore, and a candidate for
deletion if you're cleaning up dead files.
