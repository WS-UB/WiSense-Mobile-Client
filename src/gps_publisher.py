#!/usr/bin/env python3

import rospy
import websocket
import json
from time import sleep
import threading
from sensor_msgs.msg import NavSatFix
import sys
import signal

class GPSPublisher:
    def __init__(self, target_socket):
        self.closed = False
        self.gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=10)
        print(target_socket)
        self.url = target_socket

    def on_message(self, ws, message):
        # rospy.loginfo("Message received: %s", message)
        try:
            data = json.loads(message)
            lat = data.get("latitude")
            lon = data.get("longitude")
            last_known_location = data.get("lastKnownLocation")  # Corrected key name

            if lat is not None and lon is not None:
                # rospy.loginfo(f"Latitude: {lat}, Longitude: {lon}")
                self.publish_gps_data(lat, lon)
            else:
                rospy.logwarn("GPS coordinates not available.")
                
        except json.JSONDecodeError as e:
            rospy.logerr("Failed to decode JSON: %s", e)

    def on_error(self, ws, error):
        rospy.logerr("Error occurred: %s", error)

    def on_close(self, ws, close_code, reason):
        self.closed = True
        rospy.loginfo("Connection closed: %s", reason)

    def on_open(self, ws):
        rospy.loginfo("Connection established.")
        thread = threading.Thread(target=self.send_requests, args=(ws,))
        thread.start()

    def send_requests(self, ws):
        while not self.closed:
            rospy.loginfo("Sending getLastKnownLocation request")
            ws.send("getLastKnownLocation")
            sleep(1)  # 1 second sleep
        rospy.loginfo("Stopping request thread")

    def publish_gps_data(self, lat, lon):
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = "gps_link"
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = 0.0  # Default value for altitude
        self.gps_pub.publish(gps_msg)

    def connect(self):
        ws = websocket.WebSocketApp(f"{self.url}/gps",
                                    on_open=self.on_open,
                                    on_message=self.on_message,
                                    on_error=self.on_error,
                                    on_close=self.on_close)
        ws.run_forever()

def signal_handler(sig, frame):
    rospy.loginfo('Ctrl+C pressed, shutting down...')
    gps_pub.closed = True
    sys.exit(0)

if __name__ == "__main__":
    rospy.init_node('gps_publisher', anonymous=True)
    target_socket = rospy.get_param('/gps_publisher/target_socket')
    gps_pub = GPSPublisher(target_socket)
    signal.signal(signal.SIGINT, signal_handler)
    gps_pub.connect()
    rospy.spin()

