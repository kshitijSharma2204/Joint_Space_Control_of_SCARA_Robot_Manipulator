import matplotlib.pyplot as plt
import csv
  
x = []
J1_SP_y = []
J1_CP_y = []
  
with open('trajactory1.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        x.append(row[0])
        y.append(int(row[1]))
  
plt.plot(x, J1_SP_y, color = 'g', linestyle = 'dashed',
         marker = 'o',label = "Joint1 Setpoint")
  
plt.xticks(rotation = 25)
plt.xlabel('steps')
plt.ylabel('Joint Value')
plt.title('Joint trajactory', fontsize = 20)
plt.grid()
plt.legend()
plt.show()