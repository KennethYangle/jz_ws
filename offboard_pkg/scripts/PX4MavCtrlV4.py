#!/usr/bin/env python3
# coding=utf-8

import math
import numpy as np

class EarthModel():
    def __init__(self):
        self.wgs84_a = 6378137
        self.wgs84_b = 6356752.3142
        self.wgs84_f = (self.wgs84_a - self.wgs84_b) / self.wgs84_a
        self.pow_e_2 = self.wgs84_f * (2-self.wgs84_f)
 
    def lla2ecef(self, lat, lon, h):
        # (lat, lon) in degrees
        # h in meters
        lamb = math.radians(lat)
        phi = math.radians(lon)
        s = math.sin(lamb)
        N = self.wgs84_a / math.sqrt(1 - self.pow_e_2 * s * s)
    
        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
    
        x = (h + N) * cos_lambda * cos_phi
        y = (h + N) * cos_lambda * sin_phi
        z = (h + (1 - self.pow_e_2) * N) * sin_lambda
    
        return x, y, z
    
    def ecef2enu(self, x, y, z, lat0, lon0, h0):
        lamb = math.radians(lat0)
        phi = math.radians(lon0)
        s = math.sin(lamb)
        N = self.wgs84_a / math.sqrt(1 - self.pow_e_2 * s * s)
    
        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
    
        x0 = (h0 + N) * cos_lambda * cos_phi
        y0 = (h0 + N) * cos_lambda * sin_phi
        z0 = (h0 + (1 - self.pow_e_2) * N) * sin_lambda
    
        xd = x - x0
        yd = y - y0
        zd = z - z0
    
        t = -cos_phi * xd -  sin_phi * yd
    
        xEast = -sin_phi * xd + cos_phi * yd
        yNorth = t * sin_lambda  + cos_lambda * zd
        zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd
    
        return xEast, yNorth, zUp
    
    def enu2ecef(self, xEast, yNorth, zUp, lat0, lon0, h0):
        lamb = math.radians(lat0)
        phi = math.radians(lon0)
        s = math.sin(lamb)
        N = self.wgs84_a / math.sqrt(1 - self.pow_e_2 * s * s)
    
        sin_lambda = math.sin(lamb)
        cos_lambda = math.cos(lamb)
        sin_phi = math.sin(phi)
        cos_phi = math.cos(phi)
    
        x0 = (h0 + N) * cos_lambda * cos_phi
        y0 = (h0 + N) * cos_lambda * sin_phi
        z0 = (h0 + (1 - self.pow_e_2) * N) * sin_lambda
    
        t = cos_lambda * zUp - sin_lambda * yNorth
    
        zd = sin_lambda * zUp + cos_lambda * yNorth
        xd = cos_phi * t - sin_phi * xEast 
        yd = sin_phi * t + cos_phi * xEast
    
        x = xd + x0 
        y = yd + y0 
        z = zd + z0 
    
        return x, y, z
    
    def ecef2lla(self, x, y, z):
    # Convert from ECEF cartesian coordinates to 
    # latitude, longitude and height.  WGS-84
        x2 = x ** 2 
        y2 = y ** 2 
        z2 = z ** 2 
    
        self.wgs84_a = 6378137.0000    # earth radius in meters
        self.wgs84_b = 6356752.3142    # earth semiminor in meters 
        e = math.sqrt (1-(self.wgs84_b/self.wgs84_a)**2) 
        b2 = self.wgs84_b*self.wgs84_b 
        e2 = e ** 2 
        ep = e*(self.wgs84_a/self.wgs84_b) 
        r = math.sqrt(x2+y2) 
        r2 = r*r 
        E2 = self.wgs84_a ** 2 - self.wgs84_b ** 2 
        F = 54*b2*z2 
        G = r2 + (1-e2)*z2 - e2*E2 
        c = (e2*e2*F*r2)/(G*G*G) 
        s = ( 1 + c + math.sqrt(c*c + 2*c) )**(1/3) 
        P = F / (3 * (s+1/s+1)**2 * G*G) 
        Q = math.sqrt(1+2*e2*e2*P) 
        ro = -(P*e2*r)/(1+Q) + math.sqrt((self.wgs84_a*self.wgs84_a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2) 
        tmp = (r - e2*ro) ** 2 
        U = math.sqrt( tmp + z2 ) 
        V = math.sqrt( tmp + (1-e2)*z2 ) 
        zo = (b2*z)/(self.wgs84_a*V) 
    
        height = U*( 1 - b2/(self.wgs84_a*V) ) 
        
        lat = math.atan( (z + ep*ep*zo)/r ) 
    
        temp = math.atan(y/x) 
        if x >=0 :    
            long = temp 
        elif (x < 0) & (y >= 0):
            long = math.pi + temp 
        else :
            long = temp - math.pi 
    
        lat0 = lat/(math.pi/180) 
        lon0 = long/(math.pi/180) 
        h0 = height 
    
        return lat0, lon0, h0
    
    
    def lla2enu(self, lat, lon, h, lat_ref, lon_ref, h_ref):
    
        x, y, z = self.lla2ecef(lat, lon, h)
        
        return self.ecef2enu(x, y, z, lat_ref, lon_ref, h_ref)
    
    def enu2lla(self, enu, lla0):
        xEast=enu[0]
        yNorth=enu[1]
        zUp=enu[2]
        lat_ref=lla0[0]
        lon_ref=lla0[1]
        h_ref=lla0[2]

        x,y,z = self.enu2ecef(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref)
    
        return self.ecef2lla(x,y,z)
    
    def lla2ned(self, lla, lla0):
        lat=lla[0]
        lon=lla[1]
        h=lla[2]
        lat_ref=lla0[0]
        lon_ref=lla0[1]
        h_ref=lla0[2]
        xEast, yNorth, zUp=self.lla2enu(lat, lon, h, lat_ref, lon_ref, h_ref)
        return [yNorth,xEast,-zUp]
    
    
    def ned2lla(self,ned,lla0):
        xEast=ned[1]
        yNorth=ned[0]
        zUp=-ned[2]
        lat_ref=lla0[0]
        lon_ref=lla0[1]
        h_ref=lla0[2]
        return self.enu2lla(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref)

