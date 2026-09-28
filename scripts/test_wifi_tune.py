import json
from pathlib import Path
import tempfile
import unittest
import wifi_tune as cli

class FakeResponse:
    def __init__(self, status, data): self.status=status; self.body=json.dumps(data).encode()
    def read(self, limit): return self.body[:limit]

class MockHttp:
    calls=[]
    response_status=200
    timeout_next=False
    def __init__(self, host, port, timeout=None): self.host=host; self.port=port; self.timeout=timeout; self.payload=None
    def request(self, method, path, body=None, headers=None):
        self.method,self.path,self.headers=method,path,headers or {}
        self.__class__.calls.append((method,path,self.headers.get("Authorization"),body))
        self.payload=json.loads(body) if body else None
    def getresponse(self):
        if self.__class__.timeout_next:
            self.__class__.timeout_next=False
            raise TimeoutError("mock timeout")
        if self.__class__.response_status != 200:
            return FakeResponse(self.__class__.response_status,{"ok":False,"error":"DRIVE_ACTIVE"})
        if self.method=="POST":
            return FakeResponse(200,{"ok":True,"request_id":self.payload["request_id"],"boot_id":"a1b2c3d4","values":self.payload["values"]})
        if self.path.endswith("/status"):
            return FakeResponse(200,{"ok":True,"api_version":1,"boot_id":"a1b2c3d4","supported_mode":True,"rc_valid":True,"rc_age_ms":10,"ch5_off":True,"tuning_mode":1})
        return FakeResponse(200,{"ok":True,"api_version":1,"boot_id":"a1b2c3d4","values":{"U":1,"PP":5,"PD":0.12,"V":0.2}})
    def close(self): pass

class CliTests(unittest.TestCase):
    def setUp(self):
        MockHttp.calls.clear(); MockHttp.response_status=200; MockHttp.timeout_next=False
        self.original=cli.http.client.HTTPConnection
        cli.http.client.HTTPConnection=MockHttp
        self.client=cli.Client("127.0.0.1","x"*32)
    def tearDown(self): cli.http.client.HTTPConnection=self.original
    def test_commands_and_finite_values(self):
        self.assertEqual(cli.parse_terminal_line(" PP5 "),("PP",5.0))
        self.assertEqual(cli.parse_terminal_line("PD0.12"),("PD",0.12))
        self.assertEqual(cli.parse_terminal_line("SP0.045"),("SP",0.045))
        self.assertEqual(cli.parse_terminal_line("V0.2"),("V",0.2))
        self.assertEqual(cli.parse_terminal_line("PP"),("PP",None))
        with self.assertRaises(cli.LocalError): cli.parse_terminal_line("P" * 129)
        for text in ("PPnan","PPinf","PP5x","T1","U0.5","PP21"):
            with self.subTest(text=text), self.assertRaises(cli.LocalError): cli.parse_terminal_line(text)
    def test_write_gets_boot_id_and_never_retries(self):
        response=self.client.write({"PP":5})
        self.assertEqual(response["values"],{"PP":5})
        self.assertEqual([c[0] for c in MockHttp.calls],["GET","POST"])
        self.assertTrue(all(c[2]=="Bearer "+"x"*32 for c in MockHttp.calls))
    def test_timeout_does_not_retry_write(self):
        self.client.status();MockHttp.calls.clear();MockHttp.timeout_next=True
        with self.assertRaises(cli.TransportError): self.client.write({"PP":5})
        self.assertEqual([c[0] for c in MockHttp.calls],["POST"])
    def test_device_rejection_and_exit_classes(self):
        MockHttp.response_status=409
        with self.assertRaises(cli.DeviceError): self.client.status()
        MockHttp.response_status=504
        with self.assertRaises(cli.DeviceError): self.client.status()
    def test_profiles_validate_all_fields_before_write(self):
        with self.assertRaises(cli.LocalError): cli.validate_profile({"version":1,"values":{"PP":5}})
        with self.assertRaises(cli.LocalError): cli.validate_profile({"version":1,"values":{"U":1,"PP":21}})
        with self.assertRaises(cli.LocalError): cli.validate_profile({"version":1,"values":{"U":1,"wifi_password":"secret"}})
        self.assertEqual(cli.validate_profile({"version":1,"values":{"U":1,"PP":5}}),{"U":1.0,"PP":5.0})
        with self.assertRaises(ValueError): json.loads('{"version":1,"values":{"U":1,"U":0}}', object_pairs_hook=cli.json_no_duplicates)
    def test_load_profile_sends_one_atomic_batch(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"profile.json"
            path.write_text(json.dumps({"version":1,"values":{"U":1,"PP":5,"PD":0.12}}))
            response=cli.load_profile(path,self.client)
            self.assertEqual(response["values"],{"U":1,"PP":5,"PD":0.12})
            self.assertEqual([call[0] for call in MockHttp.calls],["GET","POST"])
            payload=json.loads(MockHttp.calls[-1][3])
            self.assertEqual(payload["values"],{"U":1.0,"PP":5.0,"PD":0.12})
    def test_load_profile_rejects_duplicate_keys_before_write(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"profile.json"
            path.write_text('{"version":1,"values":{"U":1,"U":1,"PP":5}}')
            with self.assertRaises(cli.LocalError): cli.load_profile(path,self.client)
            self.assertEqual(MockHttp.calls,[])
    def test_save_is_private_and_requires_force(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"stand.json"
            profile=cli.save_profile(path,self.client,False)
            self.assertEqual(profile["values"]["PP"],5)
            self.assertEqual(path.stat().st_mode&0o777,0o600)
            with self.assertRaises(cli.LocalError): cli.save_profile(path,self.client,False)
            cli.save_profile(path,self.client,True)
    def test_config_requires_private_permissions_and_token(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"config.json"
            path.write_text(json.dumps({"host":"127.0.0.1","token":"x"*32}))
            path.chmod(0o600)
            self.assertEqual(cli.load_config(path,None),("127.0.0.1","x"*32))
            path.chmod(0o644)
            with self.assertRaises(cli.LocalError): cli.load_config(path,None)
    def test_host_must_be_ipv4(self):
        with self.assertRaises(cli.LocalError): cli.Client("example.com","x")
        with self.assertRaises(cli.LocalError): cli.Client("::1","x")

if __name__=="__main__": unittest.main()
