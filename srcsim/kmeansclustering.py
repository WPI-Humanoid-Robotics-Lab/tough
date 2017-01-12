from time import time
from time import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import csv

from sklearn import metrics
from sklearn.cluster import KMeans
from sklearn.datasets import load_digits
from sklearn.preprocessing import scale

np.random.seed(42)

data = []
with open('DATA.csv') as f:
 reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
 for row in reader:
      data.append(row)
# print data
print np.shape(data)


data = data[1:][:]

data = np.array(data, dtype = 'float')

# data = scale(data)
print np.shape(data)

n_samples, n_features = data.shape
n_digits =  43                          # number of lights = 43 (dont forget)

data = data*1000
sample_size = np.shape(data)[0]


# print data[:,0:3]
estimator = KMeans(init='k-means++', n_clusters=n_digits, n_init=10)
estimator.fit(data[:,0:3])

labels = np.array(estimator.labels_, dtype = 'int')
print np.shape(labels)
print labels
#exit()
data = np.column_stack([data,labels])



error_list = []
for i in range(0,n_digits):
 error_list.append([])


for i in range(0,data.shape[0]):
  error_list[labels[i]].append([data[i,0]-data[i,3],data[i,1]-data[i,4],data[i,2]-data[i,5]])

error_x = []
error_y = []
error_z = []



median_list_x = []
median_list_y = []
median_list_z = []

sd_x = []
sd_y = []
sd_z = []



for i in range(0,n_digits):
 for j in range(0, np.shape(error_list[i])[0]):
 	error_x.append(error_list[i][j][0])  
 	error_y.append(error_list[i][j][1])
 	error_z.append(error_list[i][j][2])

 median_list_x.append(np.median(error_x))
 median_list_y.append(np.median(error_y))
 median_list_z.append(np.median(error_z))
 sd_x.append (np.std(error_x))
 sd_y.append (np.std(error_y))
 sd_z.append (np.std(error_z))
 error_x = []
 error_y = []
 error_z = []

print "median for x ", median_list_x
print "median for y ",median_list_y
print "median for z ",median_list_z
print "sd for x ",sd_x
print "sd for y ",sd_y
print "sd for z ",sd_z

print estimator.cluster_centers_
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(data[:,0],data[:,1],data[:,2])

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
