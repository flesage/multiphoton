# -*- coding: utf-8 -*-
"""
Created on Thu Aug 27 13:05:34 2015

@author: Xavier Ducharme Rivard

This class initiates a TCP connection from the client side. It is able to send
a message to the server.
"""

import socket

class TCPCommClient():
    def __init__(self):
        self.serverIP = '132.207.157.48'
        
        self.TCP_PORT = 21000
        self.BUFFER_SIZE = 1024
#        self.message
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.serverIP, self.TCP_PORT))
        
    def closeSocket(self):
        self.s.close()
        
    def sendMessage(self, msg):
#        self.message = msg        
        self.s.send(msg)


