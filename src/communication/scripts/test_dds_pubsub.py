import time
import random

from vehicles import Vehicle

from DDSPublisher import DDSPublisher
from DDSSubscriber import DDSSubscriber


vehicle = Vehicle(name="Dallara IL-15", x=200, y=200)

publisher = DDSPublisher("Vehicle", Vehicle)


def callback(data):
    print("callback", data)

subscriber = DDSSubscriber("Vehicle", Vehicle, callback)

while True:
    vehicle.x += random.choice([-1, 0, 1])
    vehicle.y += random.choice([-1, 0, 1])
    # writer.write(vehicle)
    publisher.publish(vehicle)
    print(">> Wrote vehicle")
    # print(subscriber.get_latest_message())
    time.sleep(random.random() * 0.9 + 0.1)