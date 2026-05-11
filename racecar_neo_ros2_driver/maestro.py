"""Pololu Maestro driver. Upstream: FRC4564/Maestro; ported from v1."""

from sys import version_info

import serial

PY2 = version_info[0] == 2


class Controller:
    # Maestro must be in "USB Dual Port" mode; /dev/ttyACM0 is the command port.
    def __init__(self, ttyStr='/dev/ttyACM0', device=0x0c):
        self.usb = serial.Serial(ttyStr)
        self.PololuCmd = chr(0xaa) + chr(device)
        self.Targets = [0] * 24
        self.Mins = [0] * 24
        self.Maxs = [0] * 24

    def close(self):
        self.usb.close()

    def sendCmd(self, cmd):
        cmdStr = self.PololuCmd + cmd
        if PY2:
            self.usb.write(cmdStr)
        else:
            self.usb.write(bytes(cmdStr, 'latin-1'))

    def setRange(self, chan, min_target, max_target):
        self.Mins[chan] = min_target
        self.Maxs[chan] = max_target

    def getMin(self, chan):
        return self.Mins[chan]

    def getMax(self, chan):
        return self.Maxs[chan]

    def setTarget(self, chan, target):
        if self.Mins[chan] > 0 and target < self.Mins[chan]:
            target = self.Mins[chan]
        if self.Maxs[chan] > 0 and target > self.Maxs[chan]:
            target = self.Maxs[chan]
        lsb = target & 0x7f
        msb = (target >> 7) & 0x7f
        cmd = chr(0x04) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)
        self.Targets[chan] = target

    def setSpeed(self, chan, speed):
        lsb = speed & 0x7f
        msb = (speed >> 7) & 0x7f
        cmd = chr(0x07) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    def setAccel(self, chan, accel):
        lsb = accel & 0x7f
        msb = (accel >> 7) & 0x7f
        cmd = chr(0x09) + chr(chan) + chr(lsb) + chr(msb)
        self.sendCmd(cmd)

    def getPosition(self, chan):
        cmd = chr(0x10) + chr(chan)
        self.sendCmd(cmd)
        lsb = ord(self.usb.read())
        msb = ord(self.usb.read())
        return (msb << 8) + lsb

    def isMoving(self, chan):
        if self.Targets[chan] > 0:
            if self.getPosition(chan) != self.Targets[chan]:
                return True
        return False

    def getMovingState(self):
        cmd = chr(0x13)
        self.sendCmd(cmd)
        if self.usb.read() == chr(0):
            return False
        else:
            return True

    def runScriptSub(self, subNumber):
        cmd = chr(0x27) + chr(subNumber)
        self.sendCmd(cmd)

    def stopScript(self):
        cmd = chr(0x24)
        self.sendCmd(cmd)
