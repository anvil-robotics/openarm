import subprocess, time, math

# left arm configuration
motor_set = {'1': {'type':'DM8009', 'min':200, 'max': 80},
             '2': {'type':'DM8009', 'min':190, 'max': 10},
             '3': {'type':'DM4340', 'min':90, 'max': 90},
             '4': {'type':'DM4340', 'min':0, 'max': 140},
             '5': {'type':'DM4310', 'min':90, 'max': 90},
             '6': {'type':'DM4310', 'min':45, 'max': 45},
             '7': {'type':'DM4310', 'min':90, 'max': 90},
             '8': {'type':'DM4310', 'min':45, 'max': 0}}
cmd_enable = ['python3', '-m', 'openarm.damiao', 'enable', '--motor-type', 
       'DM8009', '1', '17']
cmd_control = ['python3', '-m', 'openarm.damiao', 'control', 'pos_vel',
               '--motor-type', 'DM8009', '1', '17', '0', '1']

### Control 1 motor
while True:  # arm side selection
  try:
    arm_side = input("Right or Left? (Enter letter r or l): ")
    if (arm_side in 'rR') or (arm_side in 'lL'):
      break
  except:
    pass
  print("Please enter the correct letter.")
while True:  # motor ID input
  try:
    motorID = input("Enter the motor ID: ")
    if int(motorID) in range(1, 9):
      break
  except:
    pass
  print("No such motor exists.")
while True:  # angle input
  try:
    min_angle = motor_set[motorID]['min']
    max_angle = motor_set[motorID]['max']
    if arm_side in 'rR' and motorID in '12':  # right arm
      min_angle, max_angle = max_angle, min_angle
    print(f"Angle range: min:-{min_angle}°, max:+{max_angle}°")
    angle = float(input("Enter the angle (in degree): "))
    if angle <= max_angle and angle >= -min_angle:
      break
  except:
    pass
  print("Please enter the correct angle.")
while True:  # speed input
  try:
    speed = float(input("Enter the speed (in degree/s): "))
    if speed >= 0:
      break
  except:
    pass
  print("Please enter the correct speed.")

cmd_control[6] = motor_set[motorID]['type']
cmd_control[7], cmd_control[8] = motorID, str(int(motorID)+16)
cmd_control[9] = str(angle*math.pi/180)  # in radians
cmd_control[10] = str(speed*math.pi/180)  # in rad/s
### Move the motor
print(f"Moving J{motorID} to {angle:.2f}° ({angle*math.pi/180:.6f} rad) at {speed:.2f}°/s ({speed*math.pi/180:.6f} rad/s)......")
subprocess.run(cmd_control)
#print(cmd_control)
time.sleep(3)

exit()
