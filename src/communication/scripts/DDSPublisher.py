from cyclonedds.core import Qos, Policy
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic
from cyclonedds.util import duration
from cyclonedds.idl import IdlStruct

class DDSPublisher:
    def __init__(self, topic_name, type_):
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
        self.publisher = Publisher(self.dp)
        self.writer = DataWriter(self.publisher, self.topic)

    def publish(self, data):
        self.writer.write(data)
        # print(">> Wrote data")