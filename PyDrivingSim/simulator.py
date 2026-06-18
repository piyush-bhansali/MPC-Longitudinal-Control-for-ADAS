# Authors : Gastone Pietro Rosati Papini
# Date    : 09/08/2022
# License : MIT
import math
import signal

from pydrivingsim import World, Road
from scenarios import BasicSpeedLimit, BasicTrafficLight, OnlyVehicle, AutonomousVehicle, GetTheCoins

class GracefulKiller:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGTERM, self.exit_gracefully)

  def exit_gracefully(self, *args):
    self.kill_now = True

def main():
    Road()  # must be first so it renders behind all other objects

    # Enable this to test only single vehicle
    #av = OnlyVehicle()
    av = AutonomousVehicle()
    BasicTrafficLight()
    # Enable this to test the coins
    #GetTheCoins()
    # Enable this to test the speed limit
    BasicSpeedLimit()

    killer = GracefulKiller()
    while not killer.kill_now and World().loop:
        av.update()
        World().update()

    av.terminate()
    World().exit()

main()