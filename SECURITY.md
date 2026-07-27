# Security Policy

OpenAMRobot UI can send commands to physical robots. Treat vulnerabilities
that expose the dashboard, bypass network restrictions, leak API keys, or
permit unintended ROS commands as safety-relevant.

## Supported versions

Security fixes are made on the current default branch. No older release line
is currently maintained separately.

## Reporting a vulnerability

Do not open a public issue containing exploit details, credentials, private
network information, or a working attack against a robot.

Use GitHub's private vulnerability reporting for this repository:

1. Open the repository's **Security** tab.
2. Select **Advisories** and **Report a vulnerability**.
3. Include the affected version or commit, impact, reproduction steps, and a
   suggested mitigation if known.

If private vulnerability reporting is unavailable, contact a repository
maintainer privately through their GitHub profile and ask for a secure channel
before sending details.

## Operational security

- Only unauthenticated `AUTH_MODE=open` is implemented.
- Keep ports `5050`, `9090`, and `8080` on a trusted local network.
- Do not forward these ports directly to the public internet.
- Use firewall rules or an authenticated reverse proxy when network isolation
  alone is not sufficient.
- Never bake `.env` files or API keys into container images.
- Rotate a key immediately if it appears in source control, logs, screenshots,
  support packages, or a published image.
- The dashboard's software stop is not a replacement for a physical,
  safety-rated emergency stop.

## Disclosure

Please allow maintainers time to reproduce and fix a report before public
disclosure. Maintainers should acknowledge reports, communicate progress, and
credit reporters who want attribution.
