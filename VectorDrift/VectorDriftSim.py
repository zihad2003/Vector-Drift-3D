import math
import time

class VectorDriftSim:
    def __init__(self):
        self.mass = 1500  # kg
        self.base_mu = 0.9 # Dry asphalt
        self.gravity = 9.81
        
        self.velocity = 0 # m/s
        self.distance_to_turn = 200 # meters
        self.turn_radius = 50 # meters
        
        self.dt = 0.2 # Simulation timestep (0.2s for quicker printing)
        
        self.weather = "Dry"
        self.friction_mult = 1.0
        self.user_target_speed = 35 # m/s (approx 126 km/h)
        self.accel_force = 4500 # N max acceleration
        self.brake_force = 9000 # N maximum braking force
        
    def set_weather(self, weather):
        self.weather = weather
        if weather == "Rain":
            self.friction_mult = 0.6
        else:
            self.friction_mult = 1.0
            
    def get_safe_entry_speed(self):
        # max v = sqrt(mu * g * r)
        current_mu = self.base_mu * self.friction_mult
        return math.sqrt(current_mu * self.gravity * self.turn_radius)

    def get_stopping_distance(self, target_v=0):
        # s = (v^2 - u^2) / (2 * a)
        current_mu = self.base_mu * self.friction_mult
        max_decel = current_mu * self.gravity
        # if current velocity is less than target, stopping distance to reach target doesn't make sense, return 0
        if self.velocity <= target_v:
             return 0
        return (self.velocity**2 - target_v**2) / (2 * max_decel)

    def run_sim(self):
        print(f"=== VECTOR DRIFT: LOCAL TELEMETRY SIMULATION ===")
        print(f"Approaching Turn. Radius: {self.turn_radius}m | Weather: {self.weather} | Target Speed: {self.user_target_speed*3.6:.1f} km/h")
        print("-" * 75)
        
        while self.distance_to_turn > 0:
            safe_speed = self.get_safe_entry_speed()
            stop_dist = self.get_stopping_distance(safe_speed)
            
            # AI Controller Logic (Wait until last moment to brake)
            if self.distance_to_turn <= stop_dist * 1.05 and self.velocity > safe_speed:
                # Brake at max available limit
                current_mu = self.base_mu * self.friction_mult
                decel = min(self.brake_force / self.mass, current_mu * self.gravity)
                self.velocity -= decel * self.dt
                action = "BRAKING "
            elif self.velocity < self.user_target_speed:
                accel = self.accel_force / self.mass
                self.velocity += accel * self.dt
                action = "THROTTLE"
            else:
                action = "COASTING"
                
            self.velocity = max(0, self.velocity)
            self.distance_to_turn -= self.velocity * self.dt
            
            # Print Telemetry (only if we are still far enough to care, to avoid spam)
            if self.distance_to_turn > 0 or self.velocity > 0:
                 print(f"Dist: {max(0, self.distance_to_turn):5.1f}m | Vel: {self.velocity*3.6:5.1f} km/h | Safe Vel: {safe_speed*3.6:5.1f} km/h | Action: {action}")
                 time.sleep(0.05)
            
        print("\n--- TURN EXECUTION ---")
        safe_speed = self.get_safe_entry_speed()
        print(f"Entry Velocity: {self.velocity*3.6:.1f} km/h")
        print(f"Max Safe Velocity: {safe_speed*3.6:.1f} km/h")
        
        centripetal_needs = (self.mass * self.velocity**2) / self.turn_radius
        max_grip = (self.base_mu * self.friction_mult) * self.mass * self.gravity
        
        print(f"Required Centripetal Force: {centripetal_needs/1000:.1f} kN")
        print(f"Total Available Tire Grip:  {max_grip/1000:.1f} kN")
        
        if centripetal_needs > max_grip:
            print(">>> 💥 CRASH IMMINENT! Grip limit exceeded. 💥 <<<")
            print("The vehicle breaks traction and spins out because the user requested too much speed without braking in time!")
        else:
            print(">>> 🟢 Turn Executed Successfully. Grip maintained. 🟢 <<<")
        print("=" * 75 + "\n")


print("\n--- TEST 1: DRY ASPHALT (Fallible AI succeeds by braking) ---")
sim1 = VectorDriftSim()
sim1.set_weather("Dry")
sim1.user_target_speed = 30 # User sets sensible speed
sim1.run_sim()

print("\n--- TEST 2: SUDDEN RAIN (AI fails due to Physics constraints) ---")
sim2 = VectorDriftSim()
sim2.velocity = 30 # Started fast
sim2.distance_to_turn = 80 # Close to turn
sim2.set_weather("Rain") # Sudden rain drops grip
sim2.user_target_speed = 40 # User tells AI to speed up anyway!
sim2.run_sim()
