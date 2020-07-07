import pandas as pd
import matplotlib.pyplot as plt
import cv2
import numpy as np
import datetime
from ahrs.madgwickahrs import MadgwickAHRS
import matplotlib.dates as mdates
import os
#print(datetime.utcfromtimestamp(ts))

def fuse_imu(filename):
    df = pd.read_csv('{}.csv'.format(filename))
    df = df.fillna(0)
    df['Timestamp'] = pd.to_datetime(df['Timestamp'], unit='ms')
    names = list(df)
    data = df.values
    ahrs = MadgwickAHRS(sampleperiod=10)
    rs = []
    prev_roll, prev_pitch, prev_yaw,prev_rotate = 0,0,0,0
    for row in data:
        ahrs.update(row[8:11],row[11:14],row[14:17])
        heading = ahrs.quaternion.to_euler_angles()
        q = ahrs.quaternion._get_q()[0]
        rotate = np.rad2deg(np.arccos(q)*2)
        roll = np.rad2deg(heading[0])
        pitch = np.rad2deg(heading[1])
        yaw = np.rad2deg(heading[2])
        diff_roll = roll - prev_roll
        diff_pitch = pitch - prev_pitch
        diff_yaw = yaw - prev_yaw
        diff_rotate = rotate - prev_rotate
        rs.append({'ts':row[0],
                   'roll':roll,'pitch':pitch,'yaw':yaw,'rotate':rotate,
                   'diff_roll':diff_roll,'diff_pitch':diff_pitch,'diff_yaw':diff_yaw,'diff_rotate':diff_rotate
        })
        prev_roll,prev_pitch,prev_yaw,prev_rotate = roll,pitch,yaw,rotate
    rs = pd.DataFrame(rs)
    rs = rs.set_index('ts')
    rs.to_csv('{}_rotation.csv'.format(filename))

    # Plotting
    fig, ax = plt.subplots()
    #seconds = mdates.SecondLocator()
    #ax.xaxis.set_major_locator(seconds)

    rs[['roll','pitch','yaw','rotate']].plot(ax=ax)

    #plt.xticks( rotation=30)
    # ticks = ax.get_xticks()
    # labels = ax.get_xticklabels()
    # n = len(ticks) // 2  # Show 10 ticks.
    # ax.set_xticks(ticks[::n])
    # ax.set_xticklabels(labels[::n])
    plt.savefig('{}_angle.png'.format(filename))
    plt.clf()

    rs[['diff_roll','diff_pitch','diff_yaw','diff_rotate']].plot()
    plt.savefig('{}_diff_angle.png'.format(filename))
    return rs





# df = pd.read_csv('angularDisp_sensorlog_20200520_121113.csv')
# #print(df)

def export_video(filename):
    os.makedirs('{}_scene'.format(filename),exist_ok=True)
    video_name ='{}.mp4'.format(filename)
    ts = int(filename)/1000
    ts = datetime.datetime.utcfromtimestamp(ts)
    fps = 30
    interval = datetime.timedelta(seconds=1/fps)
    cap = cv2.VideoCapture(video_name)

    log = []
    cnt = 0
    while(cap.isOpened()):
        ret, frame = cap.read()
        if not ret:
            break
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.resize(frame,(224,300))
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        cv2.imwrite('{}_scene/{}.png'.format(filename,cnt),frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        log.append({'ts': ts, 'img_name': '{}.png'.format(cnt)})
        ts += interval
        cnt += 1
    cap.release()
    log = pd.DataFrame(log).set_index('ts')
    log.to_csv('{}_log.csv'.format(filename))
    return
# Don't try to yaw (try roll only)
def main():
    filename = '1594107091191'
    fuse_imu(filename)
    #export_video(filename)

if __name__ == '__main__':
    main()