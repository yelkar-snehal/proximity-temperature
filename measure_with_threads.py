import time
import serial
import RPi.GPIO as GPIO
from multiprocessing import Value
import queue
import csv
import threading
import joblib
import numpy as np

# set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24
GPIO_BUTTON = 25

IOT_HUB_CONNECTION_STRING = "YOUR IOT HUB CONNECTION STRING"

debug = 0

isFirstRun = True
state = None

Q = queue.Queue()

ser = serial.Serial("/dev/ttyS0", 115200)

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_BUTTON, GPIO.OUT)


def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    start_time = time.time()
    stop_time = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()

    # time difference between start and arrival
    time_elapsed = stop_time - start_time
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    ret_dist = (time_elapsed * 34300) / 2

    return ret_dist


def listen_uart_rx(state, IOT_HUB_CONNECTION_STRING):
    result = {}

    while True:
        # print("State in listen is %d" % state.value)
        # print('uart')
        received_data = ser.read()
        time.sleep(0.03)
        data_left = ser.inWaiting()
        received_data += ser.read(data_left)
        received_data = received_data.decode('utf-8')

        if received_data.find("ntemp") > -1:
            with state.get_lock():
                state.value = 1
            # print(received_data, "ntemp")
            lines = received_data.split('\n')
            tmp = lines[0].split(',')
            result['ntc'] = tmp[0].split('=')[1]
            result['ntemp'] = tmp[1].split('=')[1]
            continue

        if received_data.find("ttemp") > -1 and state.value == 1:
            # print(received_data, "temp")
            lines = received_data.split('\n')
            tmp = lines[0].split(',')
            result['ttemp'] = tmp[0].split('=')[1]
            result['avgValue'] = tmp[1].split('=')[1]

            # send signal to main thread
            Q.put(result.values())
            break


if __name__ == '__main__':
    try:
        state = Value('i', 0)
        predicter = joblib.load('new_model.joblib')

        if debug:
            # field names
            fields = ['NTC', 'NTemp', 'TTemp', 'AvgValue']

            # writing to csv file
            with open("measurements.csv", 'a') as csvfile:
                # creating a csv writer object
                csvwriter = csv.writer(csvfile)

                # writing the fields
                csvwriter.writerow(fields)

        while True:
            if debug:
                time.sleep(2)
            time.sleep(0.10)
            dist = distance()
            # print ("Measured Distance = %.1f cm" % dist)

            if dist > 10:
                continue

            print("Simulating button...")

            GPIO.output(GPIO_BUTTON, GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(GPIO_BUTTON, GPIO.LOW)

            # create thread to receive UART
            action_process = threading.Thread(target=listen_uart_rx, args=(state, IOT_HUB_CONNECTION_STRING,))
            action_process.start()
            action_process.join()

            # print(action_process.is_alive())

            # wait for signal from child thread
            try:
                # print(Q)
                rx_result = Q.get(timeout=5.0)
                Q.task_done()
            except queue.Empty:
                print("Queue was empty, measurement probably timed out!")
                with state.get_lock():
                    state.value = 0
                continue

            print('ip values from rx: ', rx_result, ' op temp: ',
                  predicter.predict(np.array(list(rx_result)).reshape(1, -1).astype(float)))

            if debug:
                print('debug')
                with open("measurements.csv", 'a') as csvfile:
                    # creating a csv writer object
                    csvwriter = csv.writer(csvfile)

                    # writing the fields
                    csvwriter.writerow(rx_result)

            time.sleep(0.8)

    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Stopped by user!")
    GPIO.cleanup()
    # myIotHubClient.disconnect()
