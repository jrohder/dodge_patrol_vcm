"""Injects firmware version information from git into the build.

The version is derived from `git describe --tags` so that tagged release
builds (e.g. v1.0.0) report exactly that version, while development builds
report something like 1.0.0-4-gabc123-dirty.
"""
import subprocess
import time

Import("env")  # noqa: F821


def _git(*args):
    try:
        return (
            subprocess.check_output(["git"] + list(args), stderr=subprocess.DEVNULL)
            .decode()
            .strip()
        )
    except Exception:
        return ""


describe = _git("describe", "--tags", "--always", "--dirty")
if not describe:
    describe = "0.0.0-dev"
version = describe.lstrip("v")
commit = _git("rev-parse", "--short", "HEAD") or "unknown"
build_date = time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime())

print(f"VCM firmware version: {version} ({commit})")

env.Append(
    CPPDEFINES=[
        ("VCM_FW_VERSION", env.StringifyMacro(version)),
        ("VCM_GIT_COMMIT", env.StringifyMacro(commit)),
        ("VCM_BUILD_DATE", env.StringifyMacro(build_date)),
    ]
)
