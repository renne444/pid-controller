# PID控制器 
# 参考 https://www.cnblogs.com/uestc-mm/p/10512333.html
import numpy as np
import matplotlib.pyplot as plt

time_length = 25

system_k = 1.0
system_b = 0.0

standard_in = 20.0
system_model = lambda i : system_k * i + system_b
standard_out = system_model(standard_in)

system_value = []
error_value = []
system_value.append(70)
system_value.append(75)
error_value.append(system_value[0] - standard_out)

Kp = 0.08
Ki = -0.5
Kd = -0.2

def PID_controller(target, t):
    if t > time_length:
        print("Timeout, Quit")
        return -1
    
    error_now = system_value[t] - target
    error_value.append(error_now)

    integrate_res = np.sum(error_value)
    different_res = error_now - error_value[t - 1]

    return Kp * error_now + Ki * integrate_res + Kd * different_res

for t_slice in range(1, time_length):
    pout = PID_controller(standard_out, t_slice)
    sys_out = system_model(pout)
    system_value.append(sys_out)
    print(f"sys_out[{t_slice}]={sys_out}")

plt.figure('PID_Controller_Direct_Mem')
plt.xlim(0,time_length)
# plt.ylim(0, 2 * standard_out)
# plt.plot(np.arange(0, time_length), [standard_out] * time_length)
print(system_value)
plt.plot(np.arange(0, time_length), system_value[1:])
plt.plot(np.arange(0, time_length), [standard_out] * time_length)
plt.title("pid controller")
plt.savefig("pid_controller/pid.png")
plt.show()