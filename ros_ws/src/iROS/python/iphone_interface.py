#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node

from gps_driver.msg import Customgps

import asyncio
import utm
import datetime
import traceback

class StandaloneDriver(Node):

    def __init__(self):
        super().__init__('standalone_driver')
        self.declare_parameter('port',"")
        self.publisher_ = self.create_publisher(Customgps, '/gps', 10)
        self.run_ = True
        asyncio.run(self.run())

    async def run(self):
        port = self.get_parameter('port').get_parameter_value().string_value
        sp = serial.Serial(port, 4800)

        while True:
            d = sp.readline().decode('utf-8')
            if '$GPGGA' not in d:
                continue

            try:
                utc,lat,lat_dir,lon,lon_dir,quality,sats,hdop,alt = d.split(",")[1:10]
                dd : datetime.datetime
                dd = datetime.datetime.combine(
                    datetime.datetime.now(datetime.timezone.utc).date(),
                    datetime.time(
                        hour=int(utc[0:2]),
                        minute=int(utc[2:4]),
                        second=int(utc[4:6]),
                        microsecond=int(float(utc[7:])*1e3)
                    ),
                    tzinfo=datetime.timezone.utc
                )
                
                t_sec = int(dd.timestamp()) // 1
                t_nsec = int(( dd.timestamp() % 1 ) * 1e9)
                utc = datetime.datetime.strptime(utc,'%H%M%S.%f')
                hdop = float(hdop)
                lat = ( float(lat[0:2]) + float(lat[2:]) / 60 ) * ( 1 if lat_dir == "N" else -1 )
                lon = ( float(lon[0:3]) + float(lon[3:]) / 60 ) * ( 1 if lon_dir == "N" else -1 )
                alt = float(alt)
                easting,northing,zone_num,zone_let = utm.from_latlon(lat,lon)
                self.get_logger().info(f"{d}, {t_sec}, {t_nsec}, {hdop}, {lat}, {lon}, {easting}, {northing}, {zone_num}, {zone_let}")

                msg = Customgps()
                msg.header.frame_id = "GPS1_frame"
                msg.header.stamp.sec = t_sec
                msg.header.stamp.nanosec = int(( t_nsec // 1e3 ) * 1e3)
                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = alt
                msg.utm_easting = easting
                msg.utm_northing = northing
                msg.zone = zone_num
                msg.letter = zone_let
                msg.hdop = hdop
                msg.gpgga_read = d

                self.publisher_.publish(msg)
            except:
                pass

    def on_shutdown(self):
        self.run_ = False


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = StandaloneDriver()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
