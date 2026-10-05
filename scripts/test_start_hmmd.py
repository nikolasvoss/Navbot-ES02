import contextlib
import shlex
from pathlib import Path
import subprocess
import unittest
from unittest import mock

import start_hmmd


class StartHmmdTests(unittest.TestCase):
    def test_local_and_ssh_dispatch_share_startup_payload(self):
        local = start_hmmd.parse_args(["--no-browser"])
        remote = start_hmmd.parse_args(["--ssh", "niko@cm5", "--no-browser"])
        with mock.patch.object(start_hmmd.subprocess, "run", return_value=mock.Mock(returncode=0)) as run:
            self.assertEqual(start_hmmd.run_startup(local), 0)
            self.assertEqual(start_hmmd.run_startup(remote), 0)

        self.assertEqual(run.call_args_list[0].kwargs["input"], start_hmmd.STARTUP_BASH)
        self.assertEqual(run.call_args_list[1].kwargs["input"], start_hmmd.STARTUP_BASH)
        self.assertEqual(run.call_args_list[0].args[0][0:3], ["bash", "-s", "--"])
        self.assertEqual(run.call_args_list[0].args[0][4], local.device)
        remote_command = run.call_args_list[1].args[0][-1]
        self.assertEqual(shlex.split(remote_command), [
            "bash", "-s", "--", "", remote.device, str(remote.baud_rate), str(remote.startup_timeout),
        ])

    def test_remote_argument_transport_quotes_shell_metacharacters(self):
        args = start_hmmd.parse_args([
            "--ssh", "cm5", "--workspace", "/tmp/ws ; touch /tmp/pwned",
            "--device", "/dev/tty ; echo injected", "--no-browser",
        ])
        command = start_hmmd.startup_command(args)
        self.assertIn("'/tmp/ws ; touch /tmp/pwned'", command[-1])
        self.assertEqual(shlex.split(command[-1]), [
            "bash", "-s", "--", "/tmp/ws ; touch /tmp/pwned", "/dev/tty ; echo injected",
            "115200", "30",
        ])

    def test_workspace_defaults_by_execution_target(self):
        local = start_hmmd.parse_args([])
        remote = start_hmmd.parse_args(["--ssh", "cm5"])
        self.assertEqual(start_hmmd.workspace_for(local), str(Path(start_hmmd.__file__).resolve().parent.parent))
        self.assertEqual(start_hmmd.workspace_for(remote), "")
        self.assertIn('"$HOME/Navbot-ES02-cm5-hmmd"', start_hmmd.STARTUP_BASH)
        self.assertEqual(start_hmmd.workspace_for(start_hmmd.parse_args(["--workspace", "/custom"])), "/custom")

    def test_rejects_malformed_ssh_destinations_and_numeric_options(self):
        for target in ("", "-oProxyCommand=bad", "host;id", "user@host name", "host/dir", "a@b@c"):
            with self.subTest(target=target), self.assertRaises(SystemExit):
                start_hmmd.parse_args(["--ssh", target])
        for option in (("--baud-rate", "0"), ("--startup-timeout", "-1"), ("--workspace", "")):
            with self.subTest(option=option), self.assertRaises(SystemExit):
                start_hmmd.parse_args(list(option))

    def test_preflight_checks_both_forwarded_ports_before_startup(self):
        checks = []

        class Probe:
            def __init__(self):
                self.port = None

            def setsockopt(self, level, option, value):
                checks.append(("reuseaddr", level, option, value))

            def bind(self, address):
                self.port = address[1]
                checks.append(self.port)

            def __enter__(self):
                return self

            def __exit__(self, *_):
                return False

        with mock.patch.object(start_hmmd.socket, "socket", side_effect=Probe), \
                mock.patch.object(start_hmmd, "run_startup", return_value=0) as startup, \
                mock.patch.object(start_hmmd, "run_tunnel", return_value=0):
            self.assertEqual(start_hmmd.main(["--ssh", "cm5", "--no-browser"]), 0)
        self.assertEqual([check for check in checks if isinstance(check, int)], [8080, 9090])
        self.assertEqual(len([check for check in checks if isinstance(check, tuple) and check[0] == "reuseaddr"]), 2)
        startup.assert_called_once_with(mock.ANY)

    def test_preflight_bind_refusal_does_not_start_remote_services(self):
        for refused_port in (8080, 9090):
            calls = []

            class Probe:
                def setsockopt(self, *_):
                    return None

                def bind(self, address):
                    calls.append(address[1])
                    if address[1] == refused_port:
                        raise OSError(98, "in use")

                def __enter__(self):
                    return self

                def __exit__(self, *_):
                    return False

            with mock.patch.object(start_hmmd.socket, "socket", side_effect=Probe), \
                    mock.patch.object(start_hmmd, "run_startup") as startup:
                self.assertEqual(start_hmmd.main(["--ssh", "cm5", "--no-browser"]), 1)
            startup.assert_not_called()
            self.assertEqual(calls[-1], refused_port)

    def test_startup_failure_and_timeout_are_reported(self):
        args = start_hmmd.parse_args(["--no-browser"])
        with mock.patch.object(start_hmmd.subprocess, "run", return_value=mock.Mock(returncode=7)):
            self.assertEqual(start_hmmd.run_startup(args), 7)
        with mock.patch.object(start_hmmd.subprocess, "run", side_effect=subprocess.TimeoutExpired("bash", 10)):
            self.assertEqual(start_hmmd.run_startup(args), 1)
        with mock.patch.object(start_hmmd.subprocess, "run", side_effect=FileNotFoundError("bash")):
            self.assertEqual(start_hmmd.run_startup(args), 1)

    def test_tunnel_forwards_both_loopback_services(self):
        args = start_hmmd.parse_args(["--ssh", "niko@cm5", "--no-browser"])
        command = start_hmmd.tunnel_command(args)
        self.assertEqual(command.count("-L"), 2)
        self.assertIn("127.0.0.1:8080:127.0.0.1:8080", command)
        self.assertIn("127.0.0.1:9090:127.0.0.1:9090", command)
        self.assertIn("ExitOnForwardFailure=yes", command)

    def test_tunnel_timeout_and_early_exit_cleanup(self):
        args = start_hmmd.parse_args(["--ssh", "cm5", "--no-browser"])
        tunnel = mock.Mock()
        tunnel.poll.return_value = None
        with mock.patch.object(start_hmmd.subprocess, "Popen", return_value=tunnel), \
                mock.patch.object(start_hmmd, "wait_for_tunnel", return_value=False):
            self.assertEqual(start_hmmd.run_tunnel(args), 1)
        tunnel.terminate.assert_called_once()
        tunnel.wait.assert_called()

        tunnel = mock.Mock()
        tunnel.poll.return_value = 255
        with mock.patch.object(start_hmmd.subprocess, "Popen", return_value=tunnel), \
                mock.patch.object(start_hmmd, "wait_for_tunnel", return_value=False):
            self.assertEqual(start_hmmd.run_tunnel(args), 255)
        tunnel.terminate.assert_not_called()

    def test_wait_for_tunnel_requires_both_ports(self):
        tunnel = mock.Mock()
        tunnel.poll.return_value = None
        connected = {8080}
        attempted = []

        def connect(address, timeout):
            attempted.append(address[1])
            if address[1] not in connected:
                raise OSError("not forwarded")
            return contextlib.nullcontext()

        ticks = iter([0, 0, 0, 0])
        with mock.patch.object(start_hmmd.socket, "create_connection", side_effect=connect), \
                mock.patch.object(start_hmmd.time, "monotonic", side_effect=lambda: next(ticks)), \
                mock.patch.object(start_hmmd.time, "sleep") as sleep:
            self.assertFalse(start_hmmd.wait_for_tunnel(tunnel, timeout=0))
            self.assertEqual(attempted, [8080, 9090])
            sleep.assert_not_called()

    def test_tunnel_interrupt_closes_only_the_tunnel(self):
        args = start_hmmd.parse_args(["--ssh", "cm5", "--no-browser"])
        tunnel = mock.Mock()
        tunnel.poll.return_value = None
        tunnel.wait.side_effect = [KeyboardInterrupt, 0]
        with mock.patch.object(start_hmmd.subprocess, "Popen", return_value=tunnel), \
                mock.patch.object(start_hmmd, "wait_for_tunnel", return_value=True):
            self.assertEqual(start_hmmd.run_tunnel(args), 0)
        tunnel.terminate.assert_called_once()
        tunnel.wait.assert_called()

    def test_shell_payload_has_valid_bash_syntax(self):
        result = subprocess.run(["bash", "-n"], input=start_hmmd.STARTUP_BASH, text=True)
        self.assertEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
