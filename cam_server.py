

# Main control script for RC car control
# Fwd/Back, Turn, Horn, Laser, Camera tilt commands
# wasd movement, arrow key tilt
# horn: Q key, laser: E key
# view localhost at http://192.168.x.x:8080 on browser




from flask import Flask, Response, render_template_string, jsonify, request
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
import io
import threading
import RPi.GPIO as GPIO
import time
import atexit



SERVO_PIN = 26
TRIG1_PIN = 23 # Q key (Laser/Trigger)
HORN_PIN = 8   # E key (Car Horn)






# L293D Motor Pins,  Motor A = Drive Fwd/Back, Motor B = Turn Left/Right)
MOTOR_DRIVE_IN1 = 17 # L293D Pin 2 
MOTOR_DRIVE_IN2 = 18 # L293D Pin 7 
MOTOR_TURN_IN3 = 13 # L293D Pin 10 
MOTOR_TURN_IN4 = 12 # L293D Pin 15




POLL_INTERVAL_SEC = 0.05 # 50 milliseconds for hardware loop







GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)




GPIO.setup([SERVO_PIN, TRIG1_PIN, HORN_PIN], GPIO.OUT)
GPIO.setup(MOTOR_DRIVE_IN1, GPIO.OUT, initial=GPIO.LOW) 
GPIO.setup(MOTOR_DRIVE_IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(MOTOR_TURN_IN3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(MOTOR_TURN_IN4, GPIO.OUT, initial=GPIO.LOW)


GPIO.output([TRIG1_PIN, HORN_PIN], GPIO.LOW)



# Servo PWM (50Hz)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)


# Horn PWM
horn_pwm = GPIO.PWM(HORN_PIN, 400)
horn_started = False


global_state = {
    "up": False, "down": False,
    "w": False, "s": False,
    "a": False, "d": False,
    "q": False, "e": False
}
state_lock = threading.Lock()



def set_servo_duty_cycle(angle):
    """Calculates and sets the PWM duty cycle for a given angle."""
    safe_angle = max(-90.0, min(90.0, angle))
    # Duty Cycle (2.5% = -90deg, 12.5% = +90deg)
    duty = 2.5 + (safe_angle + 90) * 10 / 180
    servo_pwm.ChangeDutyCycle(duty)

def set_motor_state(in1, in2, state):
    """Sets the GPIO state for a motor pair (Forward, Backward, or Stop)."""
    if state == "FORWARD":
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif state == "BACKWARD":
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    else: # STOP
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

def hardware_loop():
    """
    Core hardware logic thread.
    Handles servo control, trigger, horn, and two-motor drive/steer.
    """
    global horn_started
    servo_angle = 0.0
    
    while True:
        loop_start_time = time.time()
        

        with state_lock:
            is_up, is_down = global_state["up"], global_state["down"]
            is_w, is_s = global_state["w"], global_state["s"]
            is_a, is_d = global_state["a"], global_state["d"]
            is_q, is_e = global_state["q"], global_state["e"]

        motor_drive_state = "STOP"
        
        if is_w and not is_s:
            # W (Forward) = BACKWARD signal 
            motor_drive_state = "BACKWARD" 
        elif is_s and not is_w:
            # S (Backward) = FORWARD signal 
            motor_drive_state = "FORWARD" 
        
        # Safety: Stop if both W and S pressed
        if is_w and is_s:
            motor_drive_state = "STOP"

        set_motor_state(MOTOR_DRIVE_IN1, MOTOR_DRIVE_IN2, motor_drive_state)
        
        
        # --- 3. MOTOR B CONTROL (Turn Left/Right) ---
        motor_turn_state = "STOP"

        if is_a and not is_d:
            # A (Turn Left)
            motor_turn_state = "BACKWARD" #
        elif is_d and not is_a:
            # D (Turn Right)
            motor_turn_state = "FORWARD"
        
        # Stop if both A and D pressed
        if is_a and is_d:
            motor_turn_state = "STOP"

        set_motor_state(MOTOR_TURN_IN3, MOTOR_TURN_IN4, motor_turn_state)
        




        # SERVO LOGIC (Arrow Keys)
        servo_moving = False
        if is_up:
            servo_angle += 2.0
            servo_moving = True
        elif is_down:
            servo_angle -= 2.0
            servo_moving = True
            
        servo_angle = max(-90.0, min(90.0, servo_angle))
        
        if servo_moving:
            set_servo_duty_cycle(servo_angle)
        else:
            # Detach PWM signal to reduce jitter when stationary
            servo_pwm.ChangeDutyCycle(0)
            
        # Q key
        GPIO.output(TRIG1_PIN, GPIO.HIGH if is_q else GPIO.LOW)
        if is_e:
            if not horn_started:
                horn_pwm.start(50)
                horn_started = True
            # alternating freq for better sound
            horn_pwm.ChangeFrequency(400)
            time.sleep(0.025)
            horn_pwm.ChangeFrequency(450)
            time.sleep(0.025)
        else:
            if horn_started:
                horn_pwm.stop()
                horn_started = False
                

            time_elapsed = time.time() - loop_start_time
            sleep_time = POLL_INTERVAL_SEC - time_elapsed
            
            if sleep_time > 0:
                time.sleep(sleep_time)


t = threading.Thread(target=hardware_loop, daemon=True)
t.start()






# Camera
app = Flask(__name__)
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(
    main={"size": (640, 480), "format": "XRGB8888"},
    buffer_count=4,
    queue=False
))
picam2.set_controls({"FrameDurationLimits": (4000, 4000)})
picam2.start()

encoder = JpegEncoder()
class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()
    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            with self.condition:
                self.frame = buf
                self.condition.notify_all()
output = StreamingOutput()
picam2.start_recording(encoder, FileOutput(output))

def mjpeg_generator():
    while True:
        with output.condition:
            output.condition.wait()
            frame = output.frame
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")







HTML = """<!DOCTYPE html>
<html><head><title>Robot Control</title>
<style>
body{margin:0;background:#000;color:#fff;font-family:sans-serif;height:100vh;display:flex;flex-direction:column;}
img{width:100%;flex:1;object-fit:contain;}
.info{padding:15px;text-align:center;background:#222;font-size:1.2em;}
</style></head>
<body tabindex="0">
  <img src="/stream">
  <div class="info">
    **WASD**: Drive | **↑↓**: Pan | **Q**: Trigger | **E**: Horn
  </div>
<script>
const state={up:false,down:false,w:false,s:false,a:false,d:false,q:false,e:false};
let lastJSON = "";

function send(){
  const currentJSON = JSON.stringify(state);
  if(currentJSON === lastJSON) return; 
  lastJSON = currentJSON;

  fetch("/state",{
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body:currentJSON
  }).catch(()=>{});
}

document.body.onkeydown=e=>{
  const k=e.key.toLowerCase();
  
  // Movement keys
  if(k==="arrowup") state.up=true;
  else if(k==="arrowdown") state.down=true;
  else if(k==="w") state.w=true;
  else if(k==="s") state.s=true;
  else if(k==="a") state.a=true;
  else if(k==="d") state.d=true;
  
  // Auxiliary keys
  else if(k==="q") state.q=true;
  else if(k==="e") state.e=true;
  
  send();
  // Prevent scrolling
  if(["arrowup","arrowdown","w","s","a","d"].includes(k)) e.preventDefault();
};

document.body.onkeyup=e=>{
  const k=e.key.toLowerCase();
  
  // Movement keys
  if(k==="arrowup") state.up=false;
  else if(k==="arrowdown") state.down=false;
  else if(k==="w") state.w=false;
  else if(k==="s") state.s=false;
  else if(k==="a") state.a=false;
  else if(k==="d") state.d=false;
  
  // Auxiliary keys
  else if(k==="q") state.q=false;
  else if(k==="e") state.e=false;
  
  send();
};

window.onblur=()=>{
  // Reset all control states if the browser window loses focus
  state.up=false; state.down=false; state.w=false; state.s=false; 
  state.a=false; state.d=false; state.q=false; state.e=false;
  send();
};
</script>
</body></html>"""

@app.route("/")
def index():
    return render_template_string(HTML)

@app.route("/stream")
def stream():
    return Response(mjpeg_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/state", methods=["POST"])
def update_state():
    try:
        data = request.get_json(force=True)
        with state_lock:
            global_state.update({
                "up": bool(data.get("up")), "down": bool(data.get("down")),
                "w": bool(data.get("w")), "s": bool(data.get("s")),
                "a": bool(data.get("a")), "d": bool(data.get("d")),
                "q": bool(data.get("q")), "e": bool(data.get("e")),
            })
            
    except:
        pass
    return jsonify(success=True)






def cleanup():
    # Stop PWM and reset ALL GPIO pins on exit
    servo_pwm.stop()
    horn_pwm.stop()
    GPIO.cleanup()
atexit.register(cleanup)

if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=8080, threaded=True)
    finally:
        cleanup()
