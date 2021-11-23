from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.ball_prediction_analysis import find_matching_slice, find_slice_at_time
from util.boost_pad_tracker import BoostPadTracker
from util.drive import steer_toward_target
from util.sequence import Sequence, ControlStep
from util.vec import Vec3


class MyBot(BaseAgent):

    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.active_sequence: Sequence = None
        self.boost_pad_tracker = BoostPadTracker()
        self.friends = []
        self.foes = []
        self.my_car = []
        self.enemy_goal = []

    def initialize_agent(self):
        # Set up information about the boost pads now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())
        
    def update_player_lists(self,packet: GameTickPacket):
        self.foes = [packet.game_cars[i] for i in range(packet.num_cars) if packet.game_cars[i].team != self.team and i != self.index]
        self.friends = [packet.game_cars[i] for i in range(packet.num_cars) if packet.game_cars[i].team == self.team]
        self.my_car = packet.game_cars[self.index]
        self.enemy_goal = [goal.location for goal in self.get_field_info().goals if goal.team_num != self.team]
        #print(self.get_field_info().goals)

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """

        # Keep our boost pad info updated with which pads are currently active
        self.boost_pad_tracker.update_boost_status(packet)

        # This is good to keep at the beginning of get_output. It will allow you to continue
        # any sequences that you may have started during a previous call to get_output.
        if self.active_sequence is not None and not self.active_sequence.done:
            controls = self.active_sequence.tick(packet)
            if controls is not None:
                return controls
            
        if packet.num_cars != len(self.friends)+len(self.foes)+1: self.update_player_lists(packet)

        car_location = Vec3(self.my_car.physics.location)
        car_velocity = Vec3(self.my_car.physics.velocity)
        ball_location = Vec3(packet.game_ball.physics.location)

        # By default we will chase the ball, but target_location can be changed later
        target_location = ball_location
        controls = SimpleControllerState()
        ball_prediction = self.get_ball_prediction_struct()
        ball_in_future = find_matching_slice(ball_prediction,0,lambda s: abs(s.physics.location.z) <= 100,2)
        

    

        car_to_ball : Vec3 = ball_location-car_location
        car_to_ball_direction : Vec3 = Vec3(car_to_ball.normalized())
    

        start = Vec3(((Vec3(self.enemy_goal[0])+Vec3(-800,0,0)) - ball_location).normalized())
        end = Vec3(((Vec3(self.enemy_goal[0])+Vec3(800,0,0)) - ball_location).normalized())
        direction_of_approach : Vec3  = car_to_ball_direction.clamp(start, end)
        offset_ball_location : Vec3 = (ball_location - (direction_of_approach * 92.75))

        side_of_approach_direction : int = sign(direction_of_approach.cross(Vec3(0, 0, 1)).dot(car_to_ball))
        car_to_ball_perpendicular : Vec3 = Vec3(car_to_ball.cross(Vec3(0, 0, side_of_approach_direction)).normalized())
        adjustment : Vec3 = Vec3(abs(car_to_ball.flat().ang_to(direction_of_approach.flat())) * 2560)
        target_location = Vec3(offset_ball_location + (car_to_ball_perpendicular * adjustment))
        
        distance_remaining = (target_location - car_location).length()# an ESTIMATION of how far we need to drive - this might be spot-on or fairly far off
        
        time_remaining = ball_in_future.game_seconds - packet.game_info.seconds_elapsed
        speed_required = distance_remaining / time_remaining
    
        if speed_required > 1410 and speed_required < 2300:
            controls.boost = 1
            controls.throttle = 1.0
            controls.brake = 0
        elif speed_required < 1410: 
            t = speed_required - car_velocity.x
            controls.throttle = cap((t**2) * sign(t)/1000, -1.0, 1.0)
            controls.boost = 0
            controls.brake = 0
        elif speed_required > 2300:
            controls.brake = 0
            controls.throttle = 1
                
        if packet.game_info.is_kickoff_pause:
            target_location = ball_location
            controls.boost = 1

        #target_location = ball_location     
        # Draw some things to help understand what the bot is thinking
        self.renderer.draw_line_3d(car_location, target_location, self.renderer.white())
        self.renderer.draw_string_3d(car_location, 1, 1, f'Speed: {car_velocity.length():.1f}', self.renderer.white())
        self.renderer.draw_rect_3d(target_location, 8, 8, True, self.renderer.cyan(), centered=True)

        self.renderer.draw_string_2d(10, 20, 1, 1, str(packet.game_info.is_kickoff_pause), self.renderer.black())
        self.renderer.draw_string_2d(10, 40, 1, 1, str(distance_remaining), self.renderer.white())
        self.renderer.draw_string_2d(10, 60, 1, 1, str(f'Speed: {Vec3(self.foes[0].physics.velocity).length():.1f}'), self.renderer.black())
        
        self.renderer.draw_rect_3d(offset_ball_location,20,20,True, self.renderer.red(), centered=True)
        self.renderer.draw_rect_3d(start,8,8,True, self.renderer.blue(), centered=True)
        self.renderer.draw_rect_3d(end,8,8,True, self.renderer.blue(), centered=True)
        self.renderer.draw_line_3d(car_location, direction_of_approach, self.renderer.white())
        self.renderer.draw_rect_3d(ball_in_future.physics.location,20,20,True, self.renderer.black(),centered=True)
        
        
        self.renderer.draw_string_2d(10, 80, 1, 1, str(packet.game_info.seconds_elapsed), self.renderer.black())
        self.renderer.draw_string_2d(10, 100, 1, 1, str(f"Required Speed: {speed_required}"), self.renderer.black())
        self.renderer.draw_string_2d(10, 120, 1, 1, str(car_velocity.x), self.renderer.black())
        
        self.renderer.draw_string_2d(10, 140, 1, 1, str(distance_remaining), self.renderer.black())
        self.renderer.draw_string_2d(10, 160, 1, 1, str(f"Ball Loc Z: {packet.game_ball.physics.location.z}"), self.renderer.black())
        self.renderer.draw_string_2d(10, 180, 1, 1, str(f"Speeds Line up: {5 < car_velocity.x - speed_required < 10}"), self.renderer.black())

        

        # if 750 < car_velocity.length() < 800:
        #     # We'll do a front flip if the car is moving at a certain speed.
        #     return self.begin_front_flip(packet)
        
        
        controls.steer = steer_toward_target(self.my_car, target_location)
        
        # You can set more controls if you want, like controls.boost.

        return controls

    def begin_front_flip(self, packet):
        # Send some quickchat just for fun
        self.send_quick_chat(team_only=False, quick_chat=QuickChatSelection.Information_IGotIt)

        # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
        # logic during that time because we are setting the active_sequence.
        self.active_sequence = Sequence([
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=True)),
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=False)),
            ControlStep(duration=0.2, controls=SimpleControllerState(jump=True, pitch=-1)),
            ControlStep(duration=0.8, controls=SimpleControllerState()),
        ])

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)
    
def sign(x):
#returns the sign of a number, -1, 0, +1
    if x < 0.0:
        return -1
    elif x > 0.0:
        return 1
    else:
        return 0.0
    
    
def cap(x, low, high):
    #caps/clamps a number between a low and high value
    if x < low:
        return low
    elif x > high:
        return high
    return 