from zenfence import fence
from core import Zenmav
import time
drone = Zenmav()


fence(drone, "fence_config.toml")

