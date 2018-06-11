# #!/usr/bin/env python3
#
# import xlwt
# import time
# import numpy as np
# import matplotlib.pyplot as plt
#
# # start = time.time()
# # workbook = xlwt.Workbook(encoding='ascii')
# # worksheet = workbook.add_sheet('sheet 1')
# # end = time.time()
# # frame = 0
# # data = (1, 2, 3, 4, 5)
# # data = 6
# #
# # for i in range(100):
# #     worksheet.write(frame, 1, data)
# #     frame += 1
# #
# # print(end - start)
# # workbook.save('Excel_workbook_2.xls')
# #
# # a = []
# # a.append([1, 2, 3])
# # a.append([4, 5, 6])
# # print(a[1][2])
# # print(len([[1,2], [1,2], [3,4]]))
#
# # make a little extra space between the subplots
# plt.subplots_adjust(wspace=0.5)
#
# dt = 0.01
# t = np.arange(0, 30, dt)
# nse1 = np.random.randn(len(t))                 # white noise 1
# nse2 = len(t)                 # white noise 2
# r = np.exp(-t/0.05)
#
# # cnse1 = np.convolve(nse1, r, mode='same')*dt   # colored noise 1
# # cnse2 = np.convolve(nse2, r, mode='same')*dt   # colored noise 2
#
# # two signals with a coherent part and a random part
# # s1 = 0.01*np.sin(2*np.pi*10*t) + cnse1
# # s2 = 0.01*np.sin(2*np.pi*10*t) + cnse2
#
# plt.subplot(211)
# plt.plot(t, nse1, 'r', t, nse1, 'g')
# plt.xlim(0, 5)
# plt.xlabel('time')
# plt.ylabel('s1 and s2')
# plt.grid(True)
#
# plt.subplot(212)
# cxy, f = plt.cohere(nse1, nse1, 256, 1./dt)
# plt.ylabel('coherence')
# plt.show()
print(max(2, 55))