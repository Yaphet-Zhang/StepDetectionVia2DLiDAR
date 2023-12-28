import matplotlib.pyplot as plt
import numpy as np

v1 = [5, 20, 36, 10, 75, 90]
attr = ['a', 'b', 'c', 'd', 'e', 'f']

plt.ion()
plt.figure(1)

for j in range(100):


    # fig 1
    plt.subplot(2, 2, 1)

    data = np.random.randn(1000) + j 
    plt.hist(data,bins=40,facecolor='blue',edgecolor='red')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('aaa')

    # fig 2
    plt.subplot(2, 2, 2)

    plt.bar(attr, [i + j for i in v1], width=0.4, alpha=0.8, color='red', label="v1")
    plt.legend()
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('bbb')

    plt.show()
    plt.pause(0.01) 


    plt.clf() # delet all figure 
