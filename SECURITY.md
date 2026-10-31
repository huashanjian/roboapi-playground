# Security Policy

## RoboAPI integration

This repository adds a RoboAPI integration to NVIDIA RoboLab. Report a
vulnerability according to where the affected code lives:

- RoboAPI integration code (`policies/roboapi/`, `tools/roboapi_eval/`,
  `docs/roboapi/` and `tests/test_roboapi_*.py`): use
  [Report a vulnerability](https://github.com/huashanjian/roboapi-playground/security/advisories/new)
  in this repository. Do not open a public issue.
- All other RoboLab code: follow the NVIDIA policy below.

## NVIDIA RoboLab

### Supported Versions

| Version | Supported |
|---------|-----------|
| 0.1.x   | Yes       |

### Reporting a Vulnerability

If you discover a security vulnerability in RoboLab, please report it responsibly. **Do not open a public GitHub issue.**

Instead, please email **psirt@nvidia.com** with:

- A description of the vulnerability
- Steps to reproduce
- Affected versions
- Any potential impact

You should receive an acknowledgment within 5 business days. We will work with you to understand and address the issue before any public disclosure.

### Scope

This policy covers the RoboLab source code and its Docker image. Third-party dependencies (Isaac Lab, Isaac Sim, OpenPI, etc.) should be reported to their respective maintainers.
