import matplotlib.pyplot as plt
import csv
import os

x = []
y = []

print("Hello World!")
print(os.getcwd())
with open('../data/pid_pwm_response.csv','r') as csvfile:

    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        x.append(float(row[0]))
        y.append(float(row[1]))

x[:] = [(time - x[0])/1000000 for time in x]


plt.plot(x,y, label='Loaded from file!')
plt.xlabel('x')
plt.ylabel('y')
plt.title('PWM Response from Controller')
plt.legend()
plt.show()
