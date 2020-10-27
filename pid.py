import random
import matplotlib.pyplot as plt
import numpy as np

m_totalError = 0; m_period = .02

def calculate(P, I, D, pos, tar):
    m_positionError = tar - pos
    m_prevError = m_positionError
    m_velocityError = (m_positionError - m_prevError) / m_period
    if (I != 0):
        global m_totalError
        m_totalError = m_totalError + m_positionError * m_period
    return P * m_positionError + I * m_totalError + D * m_velocityError

sv = random.randint(1,100); tv = random.randint(1,100000)

def plot(x, y):
    plt.plot(x, y)
    #plt.plot((x, y))
    plt.ylabel('Number'); plt.xlabel('Attempts')
    plt.show()

# A = [[1,1,1], [2,2,2]]

# print(A)

# np.append(A, [[3,3,3]], axis = 0)

# print(A)

attempt = 0
count = 0
a = []
attempts = []
nums = []

while True:
    print('Current Value is:', sv, ' |  Target value is:', tv, ' |  Count is:', count, ' |  Attempt is:', attempt)

    nums.append(sv)
    attempts.append(attempt)

    if sv == tv:
        count += 1
        if count == 100:
            print(a)
            plot(attempts, nums)
            break
        elif count == 1:
            a.append(attempt+1)
    elif attempt == 400:
        plot(attempts, nums)
        break
    else:
        count = 0
    
    change = calculate(.06, .25, .01, sv, tv)
    sv += change

    attempt += 1

