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

    def initialize_agent(self):
        # Set up information about the boost pads now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())
        
    def update_player_lists(self,packet: GameTickPacket):
        self.foes = [packet.game_cars[i] for i in range(packet.num_cars) if packet.game_cars[i].team != self.team and i != self.index]
        self.friends = [packet.game_cars[i] for i in range(packet.num_cars) if packet.game_cars[i].team == self.team]
        self.my_car = packet.game_cars[self.index]

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

        if car_location.dist(ball_location) > 1500:
            # We're far away from the ball, let's try to lead it a little bit
            ball_prediction = self.get_ball_prediction_struct()  # This can predict bounces, etc
            ball_in_future = find_slice_at_time(ball_prediction, packet.game_info.seconds_elapsed + 2)
            #ball_in_future = find_matching_slice(ball_prediction,0,lambda s: abs(s.physics.location.z) <= 5,2)
            # ball_in_future might be None if we don't have an adequate ball prediction right now, like during
            # replays, so check it to avoid errors.
            if ball_in_future is not None:
                target_location = Vec3(ball_in_future.physics.location)
                self.renderer.draw_line_3d(ball_location, target_location, self.renderer.cyan())
        if self.my_car.boost < 50:
            closest = self.boost_pad_tracker.boost_pads[0].location
            for boost in self.boost_pad_tracker.boost_pads:
                if boost.is_active:
                    if car_location.dist(boost.location) < car_location.dist(closest):
                        closest = boost.location
                    
            target_location = closest
        # Draw some things to help understand what the bot is thinking
        self.renderer.draw_line_3d(car_location, target_location, self.renderer.white())
        self.renderer.draw_string_3d(car_location, 1, 1, f'Speed: {car_velocity.length():.1f}', self.renderer.white())
        self.renderer.draw_rect_3d(target_location, 8, 8, True, self.renderer.cyan(), centered=True)

        self.renderer.draw_string_2d(10, 20, 1, 1, str(self.foes[0].boost), self.renderer.black())
        self.renderer.draw_string_2d(10, 40, 1, 1, str(Vec3(self.foes[0].physics.location)), self.renderer.black())
        self.renderer.draw_string_2d(10, 60, 1, 1, str(f'Speed: {Vec3(self.foes[0].physics.velocity).length():.1f}'), self.renderer.black())
        # if 750 < car_velocity.length() < 800:
        #     # We'll do a front flip if the car is moving at a certain speed.
        #     return self.begin_front_flip(packet)
        
        controls = SimpleControllerState()
        controls.steer = steer_toward_target(self.my_car, target_location)
        controls.throttle = 1.0
        controls.boost = 1
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
