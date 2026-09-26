from dataclasses import dataclass

from cyclonedds.core import Qos, Policy
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration
from cyclonedds.core import Listener

from cyclonedds.idl import IdlStruct

class CustomListener(Listener):
        def __init__(self, parent):
            super().__init__()
            self.parent = parent

        # def on_liveliness_changed(self, reader, status):
        #     print(">> Liveliness event")

        def on_data_available(self, reader):
            # print(">> Data available")
            sample = reader.take()
            self.parent.latest_message = sample
            self.parent.callback(sample)

class DDSSubscriber:
    def __init__(self, topic_name, type_, callback):
        self.topic_name = topic_name
        self.qos = Qos(
            Policy.Reliability.Reliable(duration(microseconds=60)),
            Policy.Deadline(duration(microseconds=10)),
            Policy.Durability.TransientLocal,
            Policy.History.KeepLast(10)
        )
        self.type = type_
        self.dp = DomainParticipant()
        self.topic = Topic(self.dp, self.topic_name, self.type, qos=self.qos)
        self.subscriber = Subscriber(self.dp)
        self.listener = CustomListener(self)
        self.reader = DataReader(self.subscriber, self.topic, listener=self.listener)
        self.callback = callback
        self.latest_message = None

    def get_latest_message(self):
        return self.latest_message