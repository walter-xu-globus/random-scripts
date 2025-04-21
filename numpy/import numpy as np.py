import numpy as np



mine = np.array([-571.97956190934792,610.64053637668735,305.88450619787119]).T
original = np.array([-572.03166417064506, 615.63799999999992, 305.86852077306651]).T

diff = mine - original
eePos_base_dir = np.array([-0.30901699437494751, 0, 0.95105651629515364]).T

print("|mine-original| = " , np.linalg.norm(mine-original))
print("angle between traj and diff (deg) = " , np.arccos(np.dot(diff,eePos_base_dir)/(np.linalg.norm(diff)*np.linalg.norm(eePos_base_dir)))*180/np.pi)
print("----------------------------------------------")

##-------------------------------------------------------------------------------------------------------------
mine = np.array([390.58552113417352,630.63538453112562,305.88450594934005]).T
original = np.array([390.63166417064508, 625.63799999999992, 305.86852077306651]).T

diff = mine - original
eePos_base_dir = np.array([0.30901699437494751, 0, 0.95105651629515364]).T

print("|mine-original| = " , np.linalg.norm(mine-original))
print("angle between traj and diff (deg) = " , np.arccos(np.dot(diff,eePos_base_dir)/(np.linalg.norm(diff)*np.linalg.norm(eePos_base_dir)))*180/np.pi)
print("----------------------------------------------")
##-------------------------------------------------------------------------------------------------------------

mine = np.array([133.84876406397422,1043.0260319272832,294.28223277844609]).T
original = np.array([137.39669324725969, 1040.5264433992729, 291.79944822138629]).T

diff = mine - original
eePos_base_dir = np.array([0.63549243240267717, 0.14964866445181413, 0.75746593691513098]).T

print("|mine-original| = " , np.linalg.norm(mine-original))
print("angle between traj and diff (deg) = " , np.arccos(np.dot(diff,eePos_base_dir)/(np.linalg.norm(diff)*np.linalg.norm(eePos_base_dir)))*180/np.pi)
print("----------------------------------------------")
##-------------------------------------------------------------------------------------------------------------

mine = np.array([133.84876406397422,1043.0260319272832,294.28223277844609]).T
original = np.array([137.39669324725969, 1040.5264433992729, 291.79944822138629]).T

diff = mine - original
eePos_base_dir = np.array([0.63549243240267717, 0.14964866445181413, 0.75746593691513098]).T

print("|mine-original| = " , np.linalg.norm(mine-original))
print("angle between traj and diff (deg) = " , np.arccos(np.dot(diff,eePos_base_dir)/(np.linalg.norm(diff)*np.linalg.norm(eePos_base_dir)))*180/np.pi)
print("----------------------------------------------")