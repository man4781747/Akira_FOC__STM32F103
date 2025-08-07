import math
import random

I_totalLen = 50

L_slow = [[],[]]
for t in range(I_totalLen):
    L_slow[0].append(t/10.)
    L_slow[1].append(math.sin(t/10)*360)

with open("./slow.csv", "w") as f:
    for t in range(I_totalLen):
        f.write("{},{}\n".format(L_slow[0][t], L_slow[1][t]))

L_fast = [[],[]]
for t in range(I_totalLen):
    L_fast[0].append(t/10.)
    L_fast[1].append(math.sin(t/10)*360+random.uniform(-10, 10))

with open("./fast.csv", "w") as f:
    for t in range(I_totalLen):
        f.write("{},{}\n".format(L_fast[0][t], L_fast[1][t]))

L_Mix = [[],[]]
for t in range(I_totalLen):
    if t%10 == 0:
        # F_value = L_slow[1][int(t/10)]*0.9 + L_fast[1][t]*0.1
        F_value = L_slow[1][t]
        F_old = L_slow[1][t]
    else:
        # F_value = F_value*0.9 + L_fast[1][t]*0.1
        F_value = 0.99*(F_value+(L_fast[1][t]-L_fast[1][t-1])) + 0.01*F_old
    L_Mix[0].append(t/10.)
    L_Mix[1].append(F_value)


with open("./mix.csv", "w") as f:
    for t in range(I_totalLen):
        f.write("{},{},{},{}\n".format(L_fast[0][t], L_fast[1][t], L_Mix[1][t], L_slow[1][t]))
        # if t%10 == 0:
        #     f.write("{},{},{},{}\n".format(L_fast[0][t], L_fast[1][t], L_Mix[1][t], L_slow[1][t]))
        # else:
        #     f.write("{},{},{}\n".format(L_fast[0][t], L_fast[1][t], L_Mix[1][t]))
