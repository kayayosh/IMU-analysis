#IMU Example


import streamlit as st
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy
from scipy.signal import find_peaks
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter
from numpy import eye, array, asarray
import numpy as np
import scipy.integrate as integrate
import scipy.special as special



st.title('Xsens dot IMU analysis :-)')

file = st.file_uploader("Choose a .csv file", type = 'csv')
if file is not None:
   df = pd.read_csv(file)

samp = st.number_input("Input sample rate (default value = 60 Hz", 60)


st.subheader('Uploaded Dataframe')
df
  

display = ("Acc_X", "Acc_Y", "Acc_Z", "Gyro_X", "Gyro_Y", "Gyro_Z")
channels = list(range(len(display)))
value = st.selectbox("Channel select", channels, format_func=lambda x: display[x])
name = display[value]
value = value+5


ch = df.iloc[:,value]
ch


name


#Crop raw data

st.text("Drag slider to crop data. Note: Always crop start sample first! :)")
x = st.slider('Start sample', max_value = len(ch))  # 👈 this is a widget
o = st.slider('End sample',max_value = len(ch), value= len(ch)) 
ch = ch[x:]
ch = ch[:o]
ch.reset_index(drop=True, inplace=True,)


st.text('Start sample: ' + str(x))
st.text('End sample: ' + str(o))

st.subheader('Visualize selected channel')
fig, ax = plt.subplots()
ax.plot(ch) 
ax.set_xlabel('Time (samples)')
ax.set_ylabel('Accel m/s^2)')
ax.set_title(name)
st.pyplot(fig)


#remove offset?
st.subheader('Offset removal')

yes = st.checkbox("Remove signal offset?")
if yes:

   st.text("Drag slider to select quiet standing region for offset calculation")
   x2 = st.slider('Start quiet standing sample', max_value = len(ch))  # 👈 this is a widget
   o2 = st.slider('End quiet standing sample',max_value = len(ch)) 
   QS = ch[x2:o2]

   st.text('Start sample: ' + str(x2))
   st.text('End sample: ' + str(o2))

   offset = QS.mean()
   off_rem = ch - offset
   off_rem.reset_index(drop=True, inplace=True,)

   st.text('Avg offset over period: ' + str(offset))
#visualize quiet standing region - maybe can highlight chunk
   st.subheader('Visualize quiet standing selection')
   fig, ax = plt.subplots()
   ax.plot(ch, label = "OG data") 
   ax.plot(QS, color = 'red', label = 'Offset removed')
   ax.set_xlabel('Time (samples)')
   ax.set_ylabel('Accel m/s^2)')
   ax.set_title(name)
   ax.legend('')
   st.pyplot(fig)


   

#visualize offset removed graph
   st.subheader('Graph w offset removed')
   fig, ax = plt.subplots()
   ax.plot(off_rem, color = 'pink')
   ax.plot(ch)
   ax.set_xlabel('Time (samples)')
   ax.set_ylabel('Accel m/s^2)')
   ax.set_title(name)
   st.pyplot(fig)

   signal = off_rem

else:
   signal = ch




st.subheader(' Detect peaks of signal')
#set threshold for peak detection

st.text("Input threshold for peak detection")
thres= st.number_input("Threshold for peaks (default value = 10 m/s^2", 10)


st.text('Threshold: ' + str(thres))

peaks = scipy.signal.find_peaks(signal, height=thres)
peak_pos = peaks[0]
height = peaks[1]['peak_heights'] 

st.subheader('Acceleration data plot with peak detection')
fig, ax = plt.subplots()
ax.plot(signal) 
ax.set_xlabel('Time (samples)')
ax.set_ylabel('Accel m/s^2)')
ax.set_title(name)
ax.scatter(peak_pos, height, marker = 'X', c = 'red' , linewidth = .1)
st.pyplot(fig)



st.subheader('Calculate step time?')

StepTsamp = np.diff(peak_pos)


st.subheader(StepTsamp)

#convert samples to seconds
StepT = (StepTsamp/samp)

StepT

# create array of time based on sample rate

#list_time = []
#time = [60]
#for i in range(len(signal)):
 #  list_time.append(samp)

#list_time
#time = np.cumsum(list_time)
#time



#calculate displacement from accel 
st.subheader("Integrate over acceleration to get velocity (cumtrapz)")


time = len(signal)
totalt = int(time*samp)
dt = [1/samp]*time
dt = pd.Series(dt)
cumdt = np.cumsum(dt)
cumdt


x = signal
y = x 
dx = 1/samp
vel = integrate.cumtrapz(x, cumdt)

vel

fig, ax = plt.subplots()
ax.plot(vel) 
ax.set_xlabel('Time (samples)')
ax.set_ylabel('Velocity (m/s)')
ax.set_title('Velocity')
st.pyplot(fig)

st.subheader("Integrate over velocity to get displacement (cumtrapz)")

x = vel
y = x
dx = 1/samp
y_int = integrate.cumtrapz(y, x)


disp = (dt, scipy.integrate.cumtrapz(signal, x=dt))
disp = pd.Series(disp)

disp


fig, ax = plt.subplots()
ax.plot(disp) 
ax.set_xlabel('Time (samples)')
ax.set_ylabel('Displacement (m)')
ax.set_title('Displacement')
st.pyplot(fig)


st.subheader("Next steps... orientation filter to correct free acceleration. Madgwick or ExtendedKalmanFilter")

#Madgwick filter
acc = ch

#madgwick = Madgwick()
#Q = np.zeros((num_samples, 4))      # Allocation of quaternions
#Q[0] = [1.0, 0.0, 0.0, 0.0]         # Initial attitude as a quaternion
#for t in range(1, num_samples):
 #    madgwick.Dt = new_sample_rate
  #   Q[t] = madgwick.updateIMU(Q[t-1], gyr=gyro_data[t], acc=acc_data[t])



#initialize extended Kalman filter

#dt = 0.05
#rk = ExtendedKalmanFilter(dim_x=3, dim_z=1)
#radar = RadarSim(dt, pos=0., vel=100., alt=1000.)
