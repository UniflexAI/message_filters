# Copyright 2026, Open Source Robotics Foundation, Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from bisect import insort_right
from dataclasses import dataclass
import threading
from typing import Optional

from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration
from rclpy.time import Time



class _Signal:
    def __init__(self):
        self.callbacks = {}

    def registerCallback(self, cb, *args):
        conn = len(self.callbacks)
        self.callbacks[conn] = (cb, args)
        return conn

    def signalMessage(self, *msg):
        for (cb, args) in self.callbacks.values():
            cb(*(msg + args))


@dataclass
class QueueStatus:
    active: bool
    queue_size: int
    msgs_processed: int
    msgs_dropped: int


class _EventQueue:
    def __init__(self):
        self.events = []
        self.next_ts = Time(nanoseconds=2**63 - 1)
        self.period = Duration(nanoseconds=0)
        self.active = False
        self.msgs_processed = 0
        self.msgs_dropped = 0

    def first_timestamp(self):
        if self.events:
            first_ts = self.events[0][0]
            self.next_ts = first_ts + self.period
            self.active = True
            return first_ts
        if self.active:
            return self.next_ts
        return Time(nanoseconds=2**63 - 1)

    def pop_first(self):
        self.events.pop(0)
        self.msgs_processed += 1

    def msg_dropped(self):
        self.msgs_dropped += 1

    def set_period(self, period: Duration):
        self.period = period

    def set_active(self, active: bool):
        self.active = active

    def get_status(self):
        return QueueStatus(
            active=self.active,
            queue_size=len(self.events),
            msgs_processed=self.msgs_processed,
            msgs_dropped=self.msgs_dropped,
        )


class InputAligner:
    def __init__(self, inputs=None, timeout: Duration = Duration(nanoseconds=0), node=None):
        self.timeout = timeout if isinstance(timeout, Duration) else Duration(seconds=timeout)
        self.last_in_ts = Time(nanoseconds=0)
        self.last_out_ts = Time(nanoseconds=0)
        self.name = ''
        self.lock = threading.Lock()
        self.node = node
        self.event_queues = []
        self.signals = []
        self.input_connections = []
        self.dispatch_timer = None
        if inputs is not None:
            self.connectInput(inputs)

    def connectInput(self, inputs):
        self.disconnectAll()
        self.event_queues = [_EventQueue() for _ in inputs]
        self.signals = [_Signal() for _ in inputs]
        self.input_connections = [
            input_filter.registerCallback(self.add, i)
            for i, input_filter in enumerate(inputs)
        ]

    def disconnectAll(self):
        self.input_connections = []

    def registerCallback(self, index, callback, *args):
        return self.signals[index].registerCallback(callback, *args)

    def setName(self, name):
        self.name = name

    def getName(self):
        return self.name

    def add(self, msg, queue_index):
        msg_timestamp = self._get_stamp(msg)
        with self.lock:
            event_queue = self.event_queues[queue_index]
            if msg_timestamp < self.last_out_ts:
                event_queue.msg_dropped()
                return
            if msg_timestamp > self.last_in_ts:
                self.last_in_ts = msg_timestamp
            insort_right(event_queue.events, (msg_timestamp, msg))

    def setInputPeriod(self, index, period):
        if not isinstance(period, Duration):
            period = Duration(seconds=period)
        self.event_queues[index].set_period(period)

    def getQueueStatus(self, index):
        return self.event_queues[index].get_status()

    def setupDispatchTimer(self, node, update_rate):
        if not isinstance(update_rate, Duration):
            update_rate = Duration(seconds=update_rate)
        self.dispatch_timer = node.create_timer(
            update_rate.nanoseconds / 1e9,
            self.dispatchMessages,
        )

    def dispatchMessages(self):
        with self.lock:
            if not self._inputs_available():
                return
            input_available = True
            while input_available:
                input_available = self._dispatch_first_message()

    def _inputs_available(self):
        return any(not queue.events == [] for queue in self.event_queues)

    def _dispatch_first_message(self):
        timestamps = [queue.first_timestamp() for queue in self.event_queues]
        idx = min(range(len(timestamps)), key=timestamps.__getitem__)
        return self._dispatch_first_message_for_index(idx)

    def _dispatch_first_message_for_index(self, idx):
        event_queue = self.event_queues[idx]
        if event_queue.events:
            stamp, msg = event_queue.events[0]
            self.last_out_ts = stamp
            self.signals[idx].signalMessage(msg)
            event_queue.pop_first()
            return True
        if (self.last_in_ts - event_queue.first_timestamp()) >= self.timeout:
            event_queue.set_active(False)
            return True
        return False

    def _get_stamp(self, msg):
        stamp = msg.header.stamp
        if not isinstance(stamp, TimeMsg):
            raise TypeError(f'Expected builtin_interfaces.msg.Time, got {type(stamp)}')
        return Time.from_msg(stamp)
