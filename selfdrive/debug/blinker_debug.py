import time
import struct
from panda import Panda
from hexdump import hexdump
from panda.python.isotp import isotp_send, isotp_recv

BUS = 1

panda = Panda()
panda.set_safety_mode(Panda.SAFETY_ELM327)
panda.can_clear(BUS)

ext_diag = b"\x10\x03"
tester_present = b"\x3E\x00"
blink_left_on = b"\x2F\xBC\x15\x03"
blink_left_off = b"\x2F\xBC\x15\x00"
blink_right_on = b"\x2F\xBC\x16\x03"
blink_right_off = b"\x2F\xBC\x16\x00"


addr = 0x770
isotp_send(panda, ext_diag, addr, bus=BUS)
hexdump(isotp_recv(panda, addr + 8, bus=BUS))

isotp_send(panda, tester_present, addr, bus=BUS)
hexdump(isotp_recv(panda, addr + 8, bus=BUS))

isotp_send(panda, blink_left_on, addr, bus=BUS)
hexdump(isotp_recv(panda, addr + 8, bus=BUS))

time.sleep(1)

isotp_send(panda, blink_left_off, addr, bus=BUS)
hexdump(isotp_recv(panda, addr + 8, bus=BUS))

time.sleep(1)

isotp_send(panda, blink_right_on, addr, bus=BUS)
hexdump(isotp_recv(panda, addr + 8, bus=BUS))

time.sleep(1)

isotp_send(panda, blink_right_off, addr, bus=BUS)
hexdump(isotp_recv(panda, addr + 8, bus=BUS))

time.sleep(1)

for i in range(20):
  isotp_send(panda, tester_present, addr, bus=BUS)
  hexdump(isotp_recv(panda, addr + 8, bus=BUS))
  time.sleep(1.0)
