# Code Review

## husky_drive.py
- The `/reset` endpoint directly manipulates the PyBullet world from the Flask thread. PyBullet's API is not thread-safe, so calling `reset_robot` outside the physics loop can race with `physics_loop`'s own PyBullet calls and lead to crashes or undefined behaviour. The reset request should instead post a message that the physics thread consumes. 【F:husky_drive.py†L202-L266】
- `api_reset` currently calls `reset_robot(0)` before it discovers the Husky's body ID. `0` refers to the plane, so this first reset touches the wrong body. The subsequent fallback uses `p.getBodyUniqueId`, but PyBullet does not expose that function—`getBodyUniqueId` is undefined. As a result, the list comprehension raises an `AttributeError`, and the robot never resets. Storing the Husky's ID from `setup_world` (or iterating over `range(p.getNumBodies())` directly) would avoid both issues. 【F:husky_drive.py†L226-L239】

## robot_link/robot_api.py
- `_base_url` is initialised with `os.environ.get("HUSKY_API_URL", DEFAULT_BASE_URL).rstrip("/")`. If `HUSKY_API_URL` is set to an empty string, `rstrip` returns an empty string and the `or DEFAULT_BASE_URL` fallback restores the default URL, which is reasonable. However, because `_base_url` is evaluated at import time, changing the environment after import has no effect; providing an explicit setter or allowing callers to override the base URL would improve testability.
- `shutdown` accepts a `timeout` argument but silently ignores it. Either honour the argument or remove it to prevent confusion for callers. 【F:robot_link/robot_api.py†L64-L89】

## robot_link/link_mqtt.py & link_remote.py
- Both bridges parse numeric arguments from the free-form command string using `_extract_numbers`. This works for the provided templates, but commands such as “move forward at 0.5 m/s for two seconds” (mixed numeric/word arguments) will fail silently. Consider validating against the structured `vars` payload exposed by OmniLink instead of scraping the raw command string. 【F:robot_link/link_mqtt.py†L19-L67】【F:robot_link/link_remote.py†L19-L69】

## General
- None of the long-running scripts (`husky_drive.py`, MQTT/TCP/remote bridges) support graceful shutdown (e.g., responding to SIGINT/SIGTERM by stopping loops and closing connections). Adding signal handling would make the tooling easier to manage in automation.
