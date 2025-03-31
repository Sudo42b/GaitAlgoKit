
from ucimlrepo import fetch_ucirepo 
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
# fetch dataset 
multivariate_gait_data = fetch_ucirepo(id=760) 
  
# data (as pandas dataframes) 
X = multivariate_gait_data.data.features 
y = multivariate_gait_data.data.targets 

# metadata 
# print(multivariate_gait_data.metadata) 
  
# variable information 
# print(multivariate_gait_data.variables) 

def get_data(data, 
             subject=None,
             condition=None,
             replication=None,
             leg=None, 
             joint=None,
             time=None,
             angle=None):
    """ 
    Attribute Information:
        Biomechanical analysis of human locomotion
    Reference:
        Smoothing spline analysis of variance models: 
            A new tool for the analysis of cyclic biomechanical data.
        By Nathaniel E. Helwig, K. A. Shorter, Ping Ma, E. Hsiao-Wecksler. 2016
        Published in Journal of Biomechanics
    Args:
        data (pd.DataFrame): 데이터
        subject (int): 1 = subject 1, …, 10 = subject 10 (integer)
        condition (int): 1 = unbraced, 2 = knee brace, 3 = ankle brace
        replication (int): replication: 1 = replication 1, …, 10 = replication 10 
        leg (int): leg: 1 = left, 2 = right 
        joint (int): 1 = ankle, 2 = knee, 3 = hip
        time (int): 0 = 0% gait cycle, …, 100 = 100% gait cycle
        angle (float): joint angle in degrees
    """

    # Filtering data if None all data
    filters = {
        'subject': subject,
        'condition': condition,
        'replication': replication,
        'leg': leg,
        'joint': joint,
        'time': time,
        'angle': angle
    }
    
    for key, value in filters.items():
        if value is not None:
            data = data[data[key] == value]
    return data


if __name__ == '__main__':
    # get data
    data = get_data(X, subject=1, condition=1, replication=2, leg=1, joint=1)
    print(data)
    plt.figure()
    plt.plot(data['time'], data['angle'])
    plt.legend(['angle'])
    plt.xlabel('time')
    plt.ylabel('angle')
    plt.show()