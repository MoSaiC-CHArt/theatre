names = list()
times = list()
keys = list()

names.append("HeadPitch")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([-0.185656, -0.185656, -0.185656, -0.185656, -0.185656, -0.185656, -0.185656])

names.append("HeadYaw")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([0.0199001, 0.0199001, 0.0199001, 0.0199001, 0.0199001, 0.0199001, 0.0199001])

names.append("LElbowRoll")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([-0.4034, -0.389594, -1.21335, -0.523052, -1.15813, -0.274544, -0.400332])

names.append("LElbowYaw")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([-1.17815, -1.18889, -0.813062, -0.8238, -0.8238, -0.691876, -1.18582])

names.append("LHand")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([0.2936, 0.2936, 0.2936, 0.2936, 0.2936, 0.2936, 0.2936])

names.append("LShoulderPitch")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([1.42504, -0.989472, -1.00635, -1.00635, -1.00635, -0.991006, 1.42811])

names.append("LShoulderRoll")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([0.197844, 0.661112, 0.661112, 0.661112, 0.67185, 0.682588, 0.197844])

names.append("LWristYaw")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([0.115008, 0.115008, 0.115008, 0.115008, 0.115008, 0.115008, 0.115008])

names.append("RElbowRoll")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([0.406552, 0.406552, 0.406552, 0.406552, 0.406552, 0.406552, 0.406552])

names.append("RElbowYaw")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([1.19494, 1.19494, 1.19494, 1.19494, 1.19494, 1.19494, 1.19494])

names.append("RHand")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([0.2884, 0.2884, 0.2884, 0.2884, 0.2884, 0.2884, 0.2884])

names.append("RShoulderPitch")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([1.45581, 1.45581, 1.45581, 1.45581, 1.45581, 1.45581, 1.45581])

names.append("RShoulderRoll")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([-0.208666, -0.208666, -0.208666, -0.219404, -0.219404, -0.219404, -0.219404])

names.append("RWristYaw")
times.append([0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9])
keys.append([0.082794, 0.082794, 0.082794, 0.082794, 0.082794, 0.082794, 0.082794])

import numpy as np
import pandas as pd
df = pd.DataFrame(np.transpose(keys), columns=names, index=times[0])



