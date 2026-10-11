"""Vectors reconstructed independently from the exact official client DLL."""
import sys
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"tools"))
import dt7_input

class CodecTests(unittest.TestCase):
    def test_z_vectors_from_official_crc_table(self):
        # Offline reconstruction with the published DLL's extracted CRC table.
        cases=[(0,'5a100136000000200000000000000000000000004aea'),
               (1,'5a10013600000020000000000100000000000000f56b'),
               (256,'5a100136000000200000000000010000000000009f75'),
               (32767,'5a1001360000002000000000ff7f000000000000db10'),
               (-32768,'5a10013600000020000000000080000000000000576c'),
               (-1,'5a1001360000002000000000ffff000000000000c696')]
        for z,expected in cases:
            with self.subTest(z=z):self.assertEqual(dt7_input.encode(z=z).hex(),expected)
        for z in (32768,-32769,float('nan')):
            with self.subTest(z=z),self.assertRaises(ValueError):dt7_input.encode(z=z)

    def test_official_vectors(self):
        cases = [
            ({}, "5a100136000000200000000000000000000000004aea"),
            ({"keys":1}, "5a10013600000020000000000000000001000000f1f6"),
            ({"keys":128}, "5a1001360000002000000000000000008000000024c7"),
            ({"keys":512}, "5a10013600000020000000000000000000020000f25f"),
            ({"x":120,"y":-40}, "5a100136000000207800d8ff0000000000000000697d"),
            ({"left":1}, "5a1001360000002000000000000001000000000061ee"),
            ({"right":1}, "5a100136000000200000000000000001000000000ee1"),
        ]
        for args, expected in cases:
            with self.subTest(args=args):
                self.assertEqual(dt7_input.encode(**args).hex(),expected)
    def test_invalid_values_rejected(self):
        for args in ({"keys":65536},{"x":32768},{"y":-32769},{"left":2},{"right":-1},{"x":float('nan')}):
            with self.subTest(args=args), self.assertRaises(ValueError):dt7_input.encode(**args)

class InputTests(unittest.TestCase):
    def test_hold_combo_mouse_success_and_release(self):
        state=dt7_input.InputState()
        state.key("W",True); state.key("W",True); state.key("E",True)
        state.button("left",True); state.mouse(12,-5); state.mouse(3,2)
        self.assertEqual(state.packet(),dt7_input.encode(keys=129,x=15,y=-3,left=1))
        state.sent()
        self.assertEqual(state.packet(),dt7_input.encode(keys=129,left=1))
        state.key("W",False)
        self.assertEqual(state.packet(),dt7_input.encode(keys=128,left=1))
        state.release()
        self.assertEqual(state.packet(),dt7_input.encode())

class Transport:
    def __init__(self, failure=None): self.packets=[]; self.failure=failure; self.closed=False
    def write(self, data):
        self.packets.append(data)
        if isinstance(self.failure,Exception):raise self.failure
        return self.failure if self.failure is not None else len(data)
    def close(self):self.closed=True

class SenderTests(unittest.TestCase):
    def test_heartbeat_advances_while_key_held_and_skips_zero(self):
        transport=Transport(); sender=dt7_input.Sender(transport,lambda row:None)
        state=dt7_input.InputState(); state.key('W',True)
        sender.send(state); sender.send(state)
        self.assertEqual(transport.packets,[dt7_input.encode(keys=1,z=1),dt7_input.encode(keys=1,z=2)])
        sender.sequence=65534
        sender.send(state); sender.send(state)
        self.assertEqual(transport.packets[-2:],[dt7_input.encode(keys=1,z=-1),dt7_input.encode(keys=1,z=1)])
        sender.close(state); self.assertEqual(transport.packets[-1],dt7_input.encode())
    def test_mouse_clears_after_write_and_normal_exit_releases(self):
        transport=Transport(); records=[]
        sender=dt7_input.Sender(transport,records.append)
        state=dt7_input.InputState(); state.key("F",True); state.mouse(9,-8)
        sender.send(state); sender.send(state); sender.close(state)
        self.assertEqual(transport.packets,[dt7_input.encode(keys=512,x=9,y=-8,z=1),dt7_input.encode(keys=512,z=2),dt7_input.encode()])
        self.assertTrue(transport.closed)
        self.assertEqual(records[-1]["event"],"release-write-completed")
    def test_short_write_timeout_disconnect_stop_without_retry(self):
        for failure in (3,TimeoutError("timeout"),OSError("disconnected")):
            with self.subTest(failure=failure):
                transport=Transport(failure); records=[]; sender=dt7_input.Sender(transport,records.append)
                state=dt7_input.InputState(); state.mouse(5,6)
                with self.assertRaises(dt7_input.SendError):sender.send(state)
                self.assertEqual((state.x,state.y),(5,6))
                sender.close(state)
                self.assertEqual(len(transport.packets),1)
                self.assertTrue(transport.closed)
                self.assertEqual(records[-1]["event"],"release-unconfirmed")

class LiveTests(unittest.TestCase):
    def test_start_handshake_discards_input_and_pause_cancels_handshake(self):
        transport=Transport(); live=dt7_input.LiveInput(dt7_input.Sender(transport,lambda row:None))
        live.start(); live.key('W',True); live.mouse(4,5); live.button('left',True)
        self.assertFalse(live.active)
        live.tick(); self.assertFalse(live.active)
        live.tick(); self.assertTrue(live.active)
        self.assertEqual(transport.packets,[dt7_input.encode(),dt7_input.encode(z=1),dt7_input.encode(z=2)])
        live.tick(); self.assertEqual(transport.packets[-1],dt7_input.encode(z=3))
        live.key('W',True); live.button('left',True); live.tick()
        self.assertEqual(transport.packets[-1],dt7_input.encode(z=4))
        live.key('W',False); live.button('left',False)
        live.key('E',True); live.tick(); self.assertEqual(transport.packets[-1],dt7_input.encode(keys=128,z=5))
        live.start(); live.tick(); live.pause(); count=len(transport.packets); live.tick()
        self.assertEqual(len(transport.packets),count); self.assertFalse(live.active)
        self.assertEqual(transport.packets[-1],dt7_input.encode())
    def test_focus_loss_releases_and_requires_explicit_restart(self):
        transport=Transport(); records=[]; sender=dt7_input.Sender(transport,records.append)
        live=dt7_input.LiveInput(sender)
        live.key("W",True); live.tick(); self.assertEqual(transport.packets,[])
        live.key("W",False)
        live.start(); live.tick(); live.tick()
        live.key("W",True); live.key("W",True); live.button("left",True)
        live.mouse(4,-3); live.tick(); live.tick()
        self.assertEqual(transport.packets[-2:], [dt7_input.encode(keys=1,x=4,y=-3,left=1,z=3),dt7_input.encode(keys=1,left=1,z=4)])
        live.pause(); self.assertEqual(transport.packets[-1],dt7_input.encode())
        count=len(transport.packets)
        live.key("E",True); live.mouse(10,10); live.tick()
        self.assertEqual(len(transport.packets),count)
        live.start(); live.tick(); live.tick(); live.tick()
        self.assertEqual(transport.packets[-1],dt7_input.encode(z=7))
        live.close(); self.assertTrue(transport.closed)

class ReplayTests(unittest.TestCase):
    def test_replay_handshakes_before_actions(self):
        data={'duration_s':.2,'actions':[{'at':0,'type':'key','key':'W','down':True}]}
        transport=Transport(); replay=dt7_input.Replay(data,dt7_input.Sender(transport,lambda row:None))
        replay.tick(0); replay.tick(.033); replay.tick(.066)
        self.assertEqual(transport.packets,[dt7_input.encode(z=1),dt7_input.encode(z=2),dt7_input.encode(keys=1,z=3)])
        self.assertFalse(replay.tick(.266)); self.assertEqual(transport.packets[-1],dt7_input.encode())
    def test_ctrl_c_runs_release_flow(self):
        data={"duration_s":1,"actions":[{"at":0,"type":"key","key":"E","down":True}]}
        transport=Transport(); sender=dt7_input.Sender(transport,lambda row:None)
        with patch.object(dt7_input.time,"sleep",side_effect=KeyboardInterrupt),self.assertRaises(KeyboardInterrupt):
            dt7_input.run_replay(data,sender,False)
        self.assertEqual(transport.packets[-1],dt7_input.encode())
        self.assertTrue(transport.closed)
    def test_timed_actions_and_late_tick_do_not_burst(self):
        data={"duration_s":.2,"actions":[{"at":0,"type":"key","key":"W","down":True},
            {"at":.03,"type":"mouse","x":12,"y":-6},
            {"at":.1,"type":"key","key":"W","down":False}]}
        transport=Transport(); records=[]; sender=dt7_input.Sender(transport,records.append)
        replay=dt7_input.Replay(data,sender)
        self.assertTrue(replay.tick(0))
        self.assertEqual(transport.packets[-1],dt7_input.encode(z=1))
        self.assertTrue(replay.tick(.02)); self.assertEqual(len(transport.packets),1)
        replay.tick(.033); replay.tick(.066)
        self.assertEqual(transport.packets[-1],dt7_input.encode(keys=1,z=3))
        self.assertTrue(replay.tick(.16))
        self.assertEqual(len(transport.packets),4)
        self.assertEqual(transport.packets[-1],dt7_input.encode(keys=1,x=12,y=-6,z=4))
        replay.tick(.2); self.assertEqual(transport.packets[-1],dt7_input.encode(z=5))
        self.assertFalse(replay.tick(.266)); self.assertEqual(transport.packets[-1],dt7_input.encode())
        self.assertTrue(transport.closed)
    def test_bad_duration_order_schema_and_values_rejected(self):
        invalid=[{"duration_s":181,"actions":[]}, {"duration_s":float('nan'),"actions":[]},
            {"duration_s":1,"actions":[{"at":2,"type":"mouse","x":1,"y":1}]},
            {"duration_s":1,"actions":[{"at":0,"type":"key","key":"bad","down":True}]},
            {"duration_s":1,"actions":[{"at":0,"type":"key","key":"W","down":"false"}]},
            {"duration_s":1,"actions":[{"at":.5,"type":"mouse","x":0,"y":0},{"at":.4,"type":"mouse","x":0,"y":0}]}]
        for data in invalid:
            with self.subTest(data=data),self.assertRaises(ValueError):
                dt7_input.Replay(data,dt7_input.Sender(Transport(),lambda row:None))

class CliTests(unittest.TestCase):
    def test_dry_run_does_not_enumerate_or_open_serial(self):
        with tempfile.TemporaryDirectory() as directory:
            action=Path(directory)/"actions.json"; log=Path(directory)/"log.jsonl"
            action.write_text(json.dumps({"duration_s":.1,"actions":[{"at":0,"type":"key","key":"W","down":True}]}))
            with patch.object(dt7_input,"open_device") as opener, \
                 patch.object(dt7_input,"devices") as enumerator, contextlib.redirect_stdout(io.StringIO()):
                result=dt7_input.main(["replay",str(action),"--dry-run","--log",str(log)])
            self.assertEqual(result,0)
            opener.assert_not_called(); enumerator.assert_not_called()
            rows=[json.loads(line) for line in log.read_text().splitlines()]
            writes=[row for row in rows if row['event']=='write']
            self.assertEqual(writes[0]['hex'],dt7_input.encode().hex())
            self.assertEqual(writes[1]['hex'],dt7_input.encode(z=1).hex())
            self.assertEqual(writes[2]['hex'],dt7_input.encode(z=2).hex())
            self.assertEqual(writes[3]['hex'],dt7_input.encode(keys=1,z=3).hex())
            self.assertEqual(writes[-1]['hex'],dt7_input.encode().hex())
    def test_wrong_identity_rejected_before_transport_created(self):
        class Device:
            device="COM4"; vid=0x0483; pid=0x5740; serial_number="wrong"
        with patch.object(dt7_input,"devices",return_value=[Device()]), \
             patch.object(dt7_input,"_serial_transport") as factory:
            with self.assertRaises(ValueError):dt7_input.open_device("COM4",lambda row:None)
        factory.assert_not_called()






if __name__=='__main__':unittest.main()
