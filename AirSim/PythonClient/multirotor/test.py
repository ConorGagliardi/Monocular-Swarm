import numpy as np

#------- Create u---------------
import numpy as np

a_aux = np.array([[1, 2],
                 [2, 3],
                 [3, 1],
                 [6, 3]])

print(a_aux)

vec_list = np.diff(a_aux, axis = 0)

print(vec_list)

print(tuple(np.rint(np.nanmean(vec_list, axis = 0))))

#-------------------------------
#1)
# get a as elements with index 0, 2, 4 ....
a = a_aux[::2]
#b = u[1::2] #get b as 1,3,5,....
#2)
#differentiate
ad = np.diff(a)
#bd = np.diff(b)
#3)
#ravel putting one of everyone
#u_result = np.ravel([ad,bd],'F')

#print(u_result)