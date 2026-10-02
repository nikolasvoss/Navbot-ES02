#!/usr/bin/env python3
"""Expose verified firmware uploads as a local MCP tool."""

import asyncio
import fcntl
import json
import os
import signal
import sys
from pathlib import Path

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations


PROJECT_ROOT = Path(__file__).resolve().parents[1]
UPLOADER = PROJECT_ROOT / "scripts" / "upload_firmware.py"
LOCK_PATH = Path("/tmp/navbot-flash-mcp.lock")
OUTPUT_LIMIT = 12_000
UPLOAD_TIMEOUT_SECONDS = 300

mcp = FastMCP("NavBot firmware flashing")


def _tail(output: str) -> str:
    if len(output) <= OUTPUT_LIMIT:
        return output
    return f"[earlier output omitted]\n{output[-OUTPUT_LIMIT:]}"


async def _stop_process(
    process: asyncio.subprocess.Process,
    communicate_task: asyncio.Task[tuple[bytes, None]],
) -> bytes:
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    return (await asyncio.shield(communicate_task))[0]


@mcp.tool(
    annotations=ToolAnnotations(
        readOnlyHint=False,
        destructiveHint=True,
        openWorldHint=False,
        idempotentHint=False,
    )
)
async def flash_firmware(build_dir: str = "build/flash", port: str | None = None) -> dict[str, object]:
    """Flash a previously built image and verify it.

    Flash only in explicit safe maintenance state, with balance disabled and
    motor outputs safe. NavBot agents should use this tool when their shell
    cannot access /dev. Relative build paths use the primary checkout; absolute
    worktree paths are supported. Release the serial MCP port before calling.
    """
    requested_dir = Path(build_dir).expanduser()
    resolved_dir = (
        requested_dir if requested_dir.is_absolute() else PROJECT_ROOT / requested_dir
    ).resolve()
    if not resolved_dir.is_dir():
        raise ValueError(f"build directory does not exist: {resolved_dir}")
    if not UPLOADER.is_file():
        raise RuntimeError(f"firmware uploader not found: {UPLOADER}")

    command = [sys.executable, str(UPLOADER), str(resolved_dir)]
    legacy_options = (
        resolved_dir.parent / f".{resolved_dir.name}.arduino-build" / "build.options.json"
    )
    if not (resolved_dir / "firmware-build.json").is_file() and legacy_options.is_file():
        options = json.loads(legacy_options.read_text(encoding="utf-8"))
        fqbn = options.get("fqbn") if isinstance(options, dict) else None
        if not isinstance(fqbn, str) or not fqbn.strip():
            raise ValueError(f"legacy build options have no nonempty fqbn: {legacy_options}")
        command.extend(["--fqbn", fqbn])
    if port:
        command.extend(["--port", port])

    with LOCK_PATH.open("a", encoding="utf-8") as lock_file:
        try:
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("another NavBot firmware upload is already running") from error

        try:
            process = await asyncio.create_subprocess_exec(
                *command,
                cwd=PROJECT_ROOT,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                start_new_session=True,
            )
            communicate_task = asyncio.create_task(process.communicate())
            try:
                output, _ = await asyncio.wait_for(
                    asyncio.shield(communicate_task), timeout=UPLOAD_TIMEOUT_SECONDS
                )
            except asyncio.TimeoutError as error:
                output = (await _stop_process(process, communicate_task)).decode(errors="replace")
                raise TimeoutError(
                    f"firmware upload exceeded {UPLOAD_TIMEOUT_SECONDS} seconds\n{_tail(output)}"
                ) from error
            except BaseException:
                await _stop_process(process, communicate_task)
                raise
        finally:
            fcntl.flock(lock_file, fcntl.LOCK_UN)

    output = output.decode(errors="replace")
    output = _tail(output)
    if process.returncode != 0:
        raise RuntimeError(f"firmware upload failed with exit code {process.returncode}\n{output}")
    return {"success": True, "exit_code": process.returncode, "output": output}


if __name__ == "__main__":
    mcp.run(transport="stdio")
