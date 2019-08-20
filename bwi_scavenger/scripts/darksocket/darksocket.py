#!/usr/bin/env python
"""A tool for (somewhat) fast, socket-based offboard darknet computing. A single
TCP connection is made between the robot (the server) and an offboard GPU
machine (the client).

Usage
-----

The client loop is already exists. You initiate it with

$> python3 darksocket.py client

A file named darksocket.conf should exist in the same location as this script
on both client and server. It looks like the following:

darknet_path=xxx   # Path to darknet binary
cfg_path=xxx       # Path to network cfg file
weights_path=xxx   # Path to network weights file
meta_path=xxx      # Path to network metadata file
recv_img_path=xxx  # Path to save received images to, including file name
host=xxx           # Server hostname or IP
port=xxx           # Server port

Server functionality is wrapped in class Server. Example usage can be seen
in server_example(...). Run the example with

$> python3 darksocket.py server_example

The server is always run before the client.

Troubleshooting
---------------

Client can't connect, or server won't bind - Make sure the port is open to TCP
on both client and server. This is done with

$> sudo ufw allow {port number}/tcp

Make sure you have permission to open that port.

Improvements
------------

There are faster offboard computing solutions. Future improvements include

- Do the image exchange with UDP rather than TCP (I initially tried this, the
  lab machines rejected it)
- Rewrite this in C to lose Python overhead (Python sockets are just a wrapper
  for C sockets)
"""
from ctypes import *
from datetime import datetime
from enum import IntEnum

import math
import os
import random
import socket
import struct
import sys
import time


################################################################################
# LOAD CONFIG
################################################################################
config = {}

try:
    with open(os.getcwd() + "/darksocket.conf", "r") as f:
        for line in f.readlines():
            pair = line.strip().split("=")
            key, value = pair[0], pair[1]
            config[key] = value
except Exception:
    print("Error: failed to open darksocket.conf")
    sys.exit(1)


################################################################################
# DARKNET PYTHON INTERFACE - Modified from
# https://github.com/pjreddie/darknet/blob/master/python/darknet.py
################################################################################
def sample(probs):
    s = sum(probs)
    probs = [a/s for a in probs]
    r = random.uniform(0, 1)
    for i in range(len(probs)):
        r = r - probs[i]
        if r <= 0:
            return i
    return len(probs)-1


def c_array(ctype, values):
    arr = (ctype*len(values))()
    arr[:] = values
    return arr


class BOX(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("w", c_float),
                ("h", c_float)]


class DETECTION(Structure):
    _fields_ = [("bbox", BOX),
                ("classes", c_int),
                ("prob", POINTER(c_float)),
                ("mask", POINTER(c_float)),
                ("objectness", c_float),
                ("sort_class", c_int)]


class IMAGE(Structure):
    _fields_ = [("w", c_int),
                ("h", c_int),
                ("c", c_int),
                ("data", POINTER(c_float))]


class METADATA(Structure):
    _fields_ = [("classes", c_int),
                ("names", POINTER(c_char_p))]


try:
    lib = CDLL(config["darknet_path"] + "/libdarknet.so", RTLD_GLOBAL)
    lib.network_width.argtypes = [c_void_p]
    lib.network_width.restype = c_int
    lib.network_height.argtypes = [c_void_p]
    lib.network_height.restype = c_int

    predict = lib.network_predict
    predict.argtypes = [c_void_p, POINTER(c_float)]
    predict.restype = POINTER(c_float)

    set_gpu = lib.cuda_set_device
    set_gpu.argtypes = [c_int]

    make_image = lib.make_image
    make_image.argtypes = [c_int, c_int, c_int]
    make_image.restype = IMAGE

    get_network_boxes = lib.get_network_boxes
    get_network_boxes.argtypes = [c_void_p, c_int, c_int, c_float, c_float, POINTER(c_int), c_int, POINTER(c_int)]
    get_network_boxes.restype = POINTER(DETECTION)

    make_network_boxes = lib.make_network_boxes
    make_network_boxes.argtypes = [c_void_p]
    make_network_boxes.restype = POINTER(DETECTION)

    free_detections = lib.free_detections
    free_detections.argtypes = [POINTER(DETECTION), c_int]

    free_ptrs = lib.free_ptrs
    free_ptrs.argtypes = [POINTER(c_void_p), c_int]

    network_predict = lib.network_predict
    network_predict.argtypes = [c_void_p, POINTER(c_float)]

    reset_rnn = lib.reset_rnn
    reset_rnn.argtypes = [c_void_p]

    load_net = lib.load_network
    load_net.argtypes = [c_char_p, c_char_p, c_int]
    load_net.restype = c_void_p

    do_nms_obj = lib.do_nms_obj
    do_nms_obj.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

    do_nms_sort = lib.do_nms_sort
    do_nms_sort.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

    free_image = lib.free_image
    free_image.argtypes = [IMAGE]

    letterbox_image = lib.letterbox_image
    letterbox_image.argtypes = [IMAGE, c_int, c_int]
    letterbox_image.restype = IMAGE

    load_meta = lib.get_metadata
    lib.get_metadata.argtypes = [c_char_p]
    lib.get_metadata.restype = METADATA

    load_image = lib.load_image_color
    load_image.argtypes = [c_char_p, c_int, c_int]
    load_image.restype = IMAGE

    rgbgr_image = lib.rgbgr_image
    rgbgr_image.argtypes = [IMAGE]

    predict_image = lib.network_predict_image
    predict_image.argtypes = [c_void_p, IMAGE]
    predict_image.restype = POINTER(c_float)
except OSError:
    pass


def classify(net, meta, im):
    out = predict_image(net, im)
    res = []
    for i in range(meta.classes):
        res.append((meta.names[i], out[i]))
    res = sorted(res, key=lambda x: -x[1])
    return res


def detect(net, meta, image, thresh=.5, hier_thresh=.5, nms=.45):
    im = load_image(image, 0, 0)
    num = c_int(0)
    pnum = pointer(num)
    predict_image(net, im)
    dets = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, None, 0, pnum)
    num = pnum[0]
    if (nms): do_nms_obj(dets, num, meta.classes, nms);

    res = []
    for j in range(num):
        for i in range(meta.classes):
            if dets[j].prob[i] > 0:
                b = dets[j].bbox
                res.append((meta.names[i], dets[j].prob[i], (b.x, b.y, b.w, b.h)))
    res = sorted(res, key=lambda x: -x[1])
    free_image(im)
    free_detections(dets, num)
    return res


################################################################################
# GENERAL UTILITIES
################################################################################
class Logger:
    def __init__(self, name):
        self.name = name

    def info(self, telemetry):
        stamp = "[%s %s]" % (datetime.now(), self.name)
        print stamp, telemetry


################################################################################
# PACKET UTILITIES
#
# Utilities for creating and decoding darksocket packets. Packets begin with a
# token byte indicating type, followed by a uint32 payload size and then the
# payload. The one exception to this is extra packets created when the TCP
# stream fragments large payloads, which are purely binary and have no metadata.
# Parsing this is handled by Stream.
################################################################################
class Packet(IntEnum):
    """Tokens used as the first byte of each packet.
    """
    NONE = 0x00
    HANDSHAKE = 0x01
    IMAGE = 0x02
    DETECTIONS = 0x03


def pack(pack, data):
    """Create a new packet.

    Parameters
    ----------
    pack : Packet
        packet type
    data : bytes, str
        payload type
    """
    b_start = struct.pack("B", pack)
    b_len = struct.pack("I", len(data))

    return b_start + b_len + data


def unpack(data):
    """Decode a packet.

    Parameters
    ----------
    data : bytes
        part of the socket stream

    Return
    ------
    tuple
        packet information (packet type, payload size, payload)
    """
    token = struct.unpack("B", data[0])[0]
    size = struct.unpack("I", data[1:5])[0]
    payload = data[5:5+size]

    return (token, size, payload)


def encode_detections(dets):
    """Encodes detection data generated by darknet's python interface.

    Parameters
    ----------
    dets : tuple
        result of detect(...)

    Return
    ------
    str
        packet payload
    """
    dat = ""

    for det in dets:
        name = det[0]
        conf = str(det[1])
        bbox = ",".join([str(x) for x in det[2]])
        dat += "$" + name + "," + conf + "," + bbox

    return dat


def parse_detections(dat):
    """Decodes the payload of a detections pacet.

    Parameters
    ----------
    dat : str
        payload

    Return
    ------
    list
        list of detection tuples (label, confidence, x, y, width, height)
    """
    dets = []

    for det in dat.split("$")[1:]:
        det = det.split(",")
        dets.append(
            (
                det[0],
                float(det[1]),
                int(float(det[2])),
                int(float(det[3])),
                int(float(det[4])),
                int(float(det[5]))
            )
        )

    return dets


class StreamEvent(IntEnum):
    NONE = 0
    PAYLOAD_INBOUND = 1
    HANDSHAKE = 2
    IMAGE = 3
    DETECTIONS = 4


class Stream:
    """Class for parsing packet information from darksocket streams.
    """
    def __init__(self, log=None):
        """Streams begin with no packet history and ready to process.

        Parameters
        ----------
        log : Logger
            log out for telemetry (optional)
        """
        self.log = log                   # Logger
        self.inbound_type = Packet.NONE  # Packet type currently being processed
        self.inbound_size = 0            # Size of inbound payload
        self.inbound_payload = False     # If a multi-packet payload is inbound
        self.buffer = ""                 # Current payload buffer

        self.detections = []
        self.handshake = ""

    def save_image_payload(self):
        if self.log is not None:
            self.log.info(
                "Image payload finished. Saving to disk..."
            )

        with open(config["recv_img_path"], "wb") as out:
            out.write(self.buffer)

        if self.log is not None:
            self.log.info("Done")

    def process(self, raw):
        if len(raw) == 0:
            return

        res = StreamEvent.NONE

        if self.inbound_payload:
            # Received part of an ongoing payload
            diff = len(self.buffer) + len(raw) - self.inbound_size

            if diff >= 0:
                # Received tail end of the payload
                tail_length = self.inbound_size - len(self.buffer)
                payload_tail = raw[:tail_length]
                self.buffer += payload_tail
                self.inbound_payload = False

                # If the payload was an image, save it
                if self.inbound_type == Packet.IMAGE:
                    self.save_image_payload()
                    res = StreamEvent.IMAGE

                # Reset stream
                self.inbound_type = Packet.NONE
                self.inbound_size = 0
                self.buffer = ""

                if diff > 0:
                    # Part of the next packet got lumped in with this one
                    self.process(raw[tail_length:])
            else:
                # Stream will continue
                self.buffer += raw
        else:
            unpacked = unpack(raw)
            self.buffer += unpacked[2]

            if len(unpacked[2]) < unpacked[1]:
                # Payload got fragmented
                self.inbound_type = unpacked[0]
                self.inbound_size = unpacked[1]
                self.inbound_payload = True
            else:
                # Payload is intact in a single packet
                if unpacked[0] == Packet.IMAGE:
                    self.save_image_payload()
                    res = StreamEvent.IMAGE
                elif unpacked[0] == Packet.DETECTIONS:
                    self.detections = parse_detections(
                        self.buffer
                    )
                    res = StreamEvent.DETECTIONS
                elif unpacked[0] == Packet.HANDSHAKE:
                    self.handshake = unpacked[2]
                    res = StreamEvent.HANDSHAKE
                self.buffer = ""

        return res


################################################################################
# CLIENT
################################################################################
def client_loop():
    """Client loop that runs on the offboard machine. Loads the network
    specified in the config file.
    """
    log = Logger("client")
    stream = Stream()

    log.info("Loading darknetwork...")

    net = load_net(
        bytes(config["cfg_path"]),
        bytes(config["weights_path"]),
        0
    )
    meta = load_meta(
        bytes(config["meta_path"])
    )

    log.info("The netscape darkens")

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    log.info("Connecting to server...")
    s.connect((config["host"], int(config["port"])))
    log.info("Connection established!")
    s.sendall(pack(Packet.HANDSHAKE, log.name))

    try:
        while True:
            raw = s.recv(1024)  # Receive 1 kb at a time
            res = stream.process(raw)
            if res == StreamEvent.IMAGE:
                # Image received; run detector and return result
                dets = detect(net, meta,
                    bytes(config["recv_img_path"])
                )
                s.sendall(pack(Packet.DETECTIONS, encode_detections(dets)))

    except KeyboardInterrupt:
        pass

    s.close()


################################################################################
# SERVER
################################################################################
class Server:
    """A convenient wrapper class for server functions.
    """
    def __init__(self, log=None, host=config["host"], port=int(config["port"])):
        """A server begins with an empty socket. No connection is attempted.

        Parameters
        ----------
        name : str
            node name (purely vanity)
        host : str
            hostname or IP (defaults to the one in the configuration file)
        port : int
            port
        """
        self.log = log
        self.ip = (host, port)
        self.stream = Stream(self.log)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn = None

    def launch(self):
        """Binds the socket to the IP.
        """
        self.socket.bind(self.ip)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    def recv(self):
        """Waits for a packet to be received. If a client has yet to connect,
        the server waits.

        Return
        ------
        StreamEvent
            what happened on this call
                StreamEvent.NONE       => nothing of interest
                StreamEvent.HANDSHAKE  => client handshake;
                                          Darksocket.stream.handshake is
                                          populated with the client name
                StreamEvent.DETECTIONS => client returned object detections;
                                          Darksocket.stream.detections is
                                          populated with the detections
        """
        if self.conn is None:
            # Wait for a connection if one has not yet been established
            if self.log is not None:
                self.log.info("Waiting for client connection...")

            self.socket.listen(1)
            self.conn, addr = self.socket.accept()
            if self.log is not None:
                self.log.info("A client is connecting from %s" % addr[0])

        # Wait for the client to send something
        raw = self.conn.recv(1024)
        res = self.stream.process(raw)

        if res == StreamEvent.HANDSHAKE:
            # Handshake received
            client = self.stream.handshake
            if self.log is not None:
                self.log.info("Handshook with %s" % client)
        elif res == StreamEvent.DETECTIONS:
            # Detections received
            dets = self.stream.detections
            if self.log is not None:
                self.log.info("Received %s detections" % len(dets))

        return res

    def send(self, packet):
        """Sends a packet to the client if one is connected.

        Parameters
        ----------
        packet : bytes
            packet, probably generated by pack(...)
        """
        if self.conn is not None:
            self.conn.sendall(packet)

    def close(self):
        """Closes the client connection if one exists.
        """
        if self.conn is not None:
            self.conn.close()


def server_example():
    """An example server interaction with the client. Initial connection is
    established with a handshake, an image is sent, and detections are received.
    """
    s = Server(Logger("server"))
    s.launch()

    res = s.recv()
    img_to_detect = "/home/bwilab/microwave.jpeg"

    if res == StreamEvent.HANDSHAKE:
        with open(img_to_detect, "rb") as f:
            image_bytes = f.read()
            s.sendall(pack(Packet.IMAGE, image_bytes))
    else:
        raise RuntimeError("Error: expected handshake packet")

    res = s.recv()

    if res == StreamEvent.DETECTIONS:
        dets = s.stream.detections
        for det in dets:
            s.log.info(det)
    else:
        raise RuntimeError("Error: expected detections packet")

    s.close()


################################################################################
# ENTRY POINT
################################################################################
if __name__ == "__main__":
    if len(sys.argv) != 2 or not sys.argv[1] in ["client", "server_example"]:
        print("Usage: python3 darksocket.py {client|server_example}")
        sys.exit(1)

    if sys.argv[1] == "client":
        client_loop()
    elif sys.argv[1] == "server_example":
        server_example()
