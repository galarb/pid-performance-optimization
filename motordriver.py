from machine import Pin, PWM
import time
import math
from ili9341 import color565  # Import color565 for color handling
from xglcd_font import XglcdFont

class motordriver:
    def __init__(self, encoder1_pin, encoder2_pin, in1_pin, in2_pin, wheel_size, display, font):
        self.encoder1 = Pin(encoder1_pin, Pin.IN)
        self.encoder1.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.encoder1_irq_handler)
        self.encoder2 = Pin(encoder2_pin, Pin.IN)

        self.in1 = Pin(in1_pin, Pin.OUT)
        self.in2 = Pin(in2_pin, Pin.OUT)
        self.pwm1 = PWM(self.in1)
        self.pwm2 = PWM(self.in2)
        self.pwm1.freq(1000)
        self.pwm2.freq(1000)
        self.wheel_size = wheel_size
        self.degrees = 0

        self.font = font  # Store the font for visualization
        self.display = display  # Store the display object

        self.last_error = 0
        self.cum_error = 0
        self.previous_time = time.ticks_ms()
        self.integral_flag = False

        self.iteration = 0  # Initialize iteration counter for plotting
        self.iteration_counter = 0  # To control how often pixels are plotted

        self.graph_x = 0  # Start drawing from the leftmost pixel
        
        
    def encoder1_irq_handler(self, pin):
        encoder1_state = self.encoder1.value()
        encoder2_state = self.encoder2.value()
        
        if encoder1_state == encoder2_state:
            self.degrees += 1
        else:
            self.degrees -= 1

        print("Degrees: ", self.degrees)

    def motgo(self, speed):
        pwm_value = int(min(max(abs(speed), 0), 100) * 10.23)  # Map -100 to 100 to 0 to 1023

        if speed > 0:
            # Forward direction
            self.pwm1.duty(pwm_value)
            self.pwm2.duty(0)
        elif speed < 0:
            # Reverse direction
            self.pwm1.duty(0)
            self.pwm2.duty(pwm_value)
        else:
            # Stop the motor
            self.pwm1.duty(0)
            self.pwm2.duty(0)    
        
    

    def PIDcalc(self, inp, sp, kp, ki, kd, color):
        current_time = time.ticks_ms()
        elapsed_time = (current_time - self.previous_time) / 1000.0

        error = sp - inp

        if error * self.last_error < 0:
            self.integral_flag = True
            self.cum_error = 0
            print("Error changed direction, resetting integral accumulator.")
        else:
            self.integral_flag = False

        if not self.integral_flag:
            self.cum_error += error * elapsed_time

        if elapsed_time > 0:
            rate_error = (error - self.last_error) / elapsed_time
            out = kp * error + ki * self.cum_error + kd * rate_error

            self.last_error = error
            self.previous_time = current_time

            out = max(-254, min(254, out))  # Clamp the output to [-254, 254]

            # Call the visualization function only after every 3rd iteration
            self.iteration_counter += 1
            if self.iteration_counter >= 3:  # Update graph every 3rd iteration
                self.plot_pid(error, color)  # Plot the current error with the specified color
                self.iteration_counter = 0  # Reset counter

            print("Degrees: ", self.degrees)
            print("PID output value: ", out)
            return out

        return 0

    
        
    
    def plot_pid(self, error, color):
        # Define the error range for scaling
        max_error = 254  # Maximum possible error value
        min_error = -254  # Minimum possible error value

        # Screen dimensions
        screen_height = 240  # Height of the screen
        screen_width = 240   # Width of the screen

        # Calculate the Y-coordinate based on the scaled error
        scaled_error = int(((error - min_error) / (max_error - min_error)) * screen_height)
        y_position = screen_height - scaled_error  # Adjust for screen origin at top-left

        # Clamp Y within screen bounds
        y_position = max(0, min(screen_height - 1, y_position))

        # Plot the pixel for the current iteration
        x_position = self.iteration % screen_width  # Wrap at screen width
        self.display.draw_pixel(x_position, y_position, color)

        # Increment the iteration counter
        self.iteration += 1


   

    def reset_graph(self):
        """Reset graph data after comparison."""
        self.previous_graph_data = self.graph_data  # Store the current graph for future comparison
        self.graph_data = []  # Clear the current graph data
        self.graph_x = 0  # Reset x position for the new run


    def reset_iterations(self):
        self.iteration = 0
        self.iteration_counter = 0



    def godegrees(self, angle, times):
        for _ in range(times):
            motspeed = self.PIDcalc(angle, self.degrees, 1, 1, 0)
            motspeed = max(-254, min(254, motspeed))  # Clamp the speed to [-254, 254]
            self.motgo(motspeed)

    
    def godegreesp(self, angle, times, kp, ki, kd, color):
        for _ in range(times):
            motspeed = self.PIDcalc(angle, self.degrees, kp, ki, kd, color)
            motspeed = max(-254, min(254, motspeed))
            self.motgo(motspeed)

    

    def gomm(self, distance, times):
        deg = (distance / (self.wheel_size * math.pi)) * 360
        self.godegrees(deg, times)
        dist_covered = (self.degrees * self.wheel_size * math.pi) / 360.0
        return dist_covered

    def gommp(self, distance, times, kp, ki, kd):
        deg = (distance / (self.wheel_size * math.pi)) * 360
        self.godegreesp(deg, times, kp, ki, kd)
        dist_covered = (self.degrees * self.wheel_size * math.pi) / 360.0

        return dist_covered
    
    
