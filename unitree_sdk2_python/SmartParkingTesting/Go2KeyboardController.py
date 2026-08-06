import sys
import time
import curses
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient


class Go2KeyboardController:
    def __init__(self, network_interface: str = "eth0"):#change to 'lo'/'eth0' for local or connected
        self.sport_client = SportClient()
        self.obstacle_client = ObstaclesAvoidClient()
 #setup sport client and obi avoid

        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        self.obstacle_client.Init()

        self.is_standing = True #may not be true on startup
        self.pressed_keys = set()
        self.obstacle_avoidance_enabled = False

    def enable_obstacle_avoidance(self):
        self.obstacle_client.UseRemoteCommandFromApi(True)
        self.obstacle_client.SwitchSet(True)  # Enable obstacle avoidance
        self.obstacle_avoidance_enabled = True
        time.sleep(0.5)

    def disable_obstacle_avoidance(self):
        self.obstacle_client.SwitchSet(False)
        self.obstacle_client.UseRemoteCommandFromApi(False)
        self.obstacle_avoidance_enabled = False

    def wave(self):
        self.pressed_keys.clear()
        self.sport_client.StopMove()
        time.sleep(0.5)
        self.sport_client.Hello()

    def stand_up(self):
        
        self.sport_client.StopMove()
        time.sleep(0.2)
        self.sport_client.StandUp()
        self.is_standing = True
        time.sleep (2.5)
        #self.sport_client.Move(0.3,0,0)


    
    def stand_down(self):
        self.pressed_keys.clear()
        self.sport_client.StopMove()
        self.sport_client.Euler(0.0, 0.0, 0.0)
        time.sleep(0.2)
        self.sport_client.StandDown()
        self.is_standing = False
        time.sleep(1.5)
        

    def run(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(50)  # 20 Hz loop
        
        stdscr.addstr(0, 0, "Initializing and standing")
        stdscr.refresh()
        self.stand_up()
        self.enable_obstacle_avoidance()
        self.sport_client.ClassicWalk(True)
        
        try:
            while True:
                try:
                    key = stdscr.getch()
                    if key != -1:
                        if key == 27:  # ESC
                            break
                        elif key == ord(' '):
                            if self.is_standing: self.stand_down()
                            else: self.stand_up()
                        elif key == ord('o'):
                            if self.obstacle_avoidance_enabled: self.disable_obstacle_avoidance()
                            else: self.enable_obstacle_avoidance()
                        elif key == ord('k'):
                            self.wave()
                        elif key in (ord('w'), ord('s'), ord('a'), ord('d'), ord('q'), ord('e'), 
                        
                                        curses.KEY_UP, curses.KEY_DOWN, curses.KEY_LEFT, curses.KEY_RIGHT):
                            if key in self.pressed_keys:
                                self.pressed_keys.remove(key)
                            else:
                                self.pressed_keys.add(key)
                except curses.error:
                    pass
                
                # 2. Compute State
                vx, vy, vyaw = 0.0, 0.0, 0.0
                roll, pitch, yaw = 0.0, 0.0, 0.0
                
                if ord('w') in self.pressed_keys:
                    if ord('s') in self.pressed_keys:
                        self.pressed_keys.remove(ord('s'))
                    vx += 0.5
                
                if ord('s') in self.pressed_keys:
                    if ord('w') in self.pressed_keys:
                        self.pressed_keys.remove(ord('w'))
                    vx -= 0.5
                    
                if ord('q') in self.pressed_keys:
                    if ord('e') in self.pressed_keys:
                        self.pressed_keys.remove(ord('e'))
                    vy += 0.3
                    
                if ord('e') in self.pressed_keys:
                    if ord('q') in self.pressed_keys:
                        self.pressed_keys.remove(ord('q'))
                    vy -= 0.3
                    
                if ord('a') in self.pressed_keys:
                    if ord('d') in self.pressed_keys:
                        self.pressed_keys.remove(ord('d'))
                    vyaw += 0.5
                    
                if ord('d') in self.pressed_keys:
                    if ord('a') in self.pressed_keys:
                        self.pressed_keys.remove(ord('a'))
                    vyaw -= 0.5
                
                if curses.KEY_UP in self.pressed_keys:
                    pitch += 0.2
                if curses.KEY_DOWN in self.pressed_keys:
                    pitch -= 0.2
                if curses.KEY_LEFT in self.pressed_keys:
                    yaw += 0.3
                if curses.KEY_RIGHT in self.pressed_keys:
                    yaw -= 0.3
                
                #Send to Robot
                

                if self.is_standing:
                    #Use ObstaclesAvoidClient.Move() when obstacle avoidance is enabled
                    if self.obstacle_avoidance_enabled:
                        self.obstacle_client.Move(vx, vy, vyaw)
                    else:
                        self.sport_client.Move(vx, vy, vyaw)
                        
                    #self.sport_client.Euler(roll, pitch, yaw)

                
                # 4. Update SSH UI
                stdscr.clear()
                stdscr.addstr(0, 0, "=== Unitree Go2 SSH Controller ===")
                status = "STANDING" if self.is_standing else "SITTING"
                avoidance = "ON" if self.obstacle_avoidance_enabled else "OFF"
                stdscr.addstr(1, 0, f"Status: {status} |Avoidance: {avoidance}| Press ESC to exit")
                stdscr.addstr(3, 0, "Move: [W/S] Fwd/Back | [Q/E] Strafe | [A/D] Rotate")
                stdscr.addstr(4, 0, "Pose: [Arrows] Look Up/Down/Left/Right")
                stdscr.addstr(5, 0, "Action: [SPACE] Stand Up/Down")
                stdscr.addstr(7, 0, f"Cmd: vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f} | pitch={pitch:.2f} yaw={yaw:.2f}")
                stdscr.refresh()
                
        finally:
            try:
                stdscr.addstr(10, 0, "Shutting down safely...")
                stdscr.refresh()
                if self.is_standing:
                    self.stand_down()
            except Exception as e:
                # If the robot disconnects during shutdown, we just print it and exit cleanly
                print(f"\nNote: Shutdown command interrupted ({e}). Robot may need manual sit")
            

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 go2_ssh_controller.py <network_interface>")
        sys.exit(1)


    network_interface = sys.argv[1]
    print(f"Network interface: {network_interface}")  # Add this
    print(f"Args count: {len(sys.argv)}")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)
    controller = Go2KeyboardController(network_interface)
    curses.wrapper(controller.run)

if __name__ == "__main__":
    main()

#run command |python3 Go2KeyboardController.py eth0