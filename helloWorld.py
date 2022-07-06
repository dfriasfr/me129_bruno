import time
t = time.localtime()
current_time = time.strftime("%H:%M:%S", t)
print("Hello world, this is your robot talking!")
print("The local time is: " + current_time)