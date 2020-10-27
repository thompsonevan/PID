from longpid import PID
import random

pid = PID(.06, .25, .01)

sv = random.randint(1,100); tv = 1000 #random.randint(1,100000)

while True:
    print('Current Value is:', sv)
    change = pid.calculate(sv, tv)
    sv += change