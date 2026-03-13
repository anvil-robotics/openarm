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

### Enabling all motors
for slaveID, motor_data in motor_set.items():
  cmd_enable[5], cmd_enable[6], cmd_enable[7] = motor_data['type'], slaveID, str(int(slaveID)+16)
  subprocess.run(cmd_enable)
  #print(*cmd_enable)
time.sleep(1)

### Zeroing all motors
for slaveID, motor_data in motor_set.items():
  cmd_control[6] = motor_data['type']
  cmd_control[7], cmd_control[8] = slaveID, str(int(slaveID)+16)
  cmd_control[9] = '0' # zero degree
  cmd_control[10] = '1' # 1 rad/s
  subprocess.run(cmd_control)
  #print(*cmd_control)
time.sleep(1)
print("All motors are activated and returned to zero position.")
exit()
