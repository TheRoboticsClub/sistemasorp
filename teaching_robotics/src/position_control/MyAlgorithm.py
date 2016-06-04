import time
from math import atan2, cos, pi, acos, sin
from sensors import sensor

class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor
	self.positions = [[0, -5], [-5, 0], [0, 5], [5, 0], [10, 0], [0, 0]]
	self.index = 0
	self.speed = 2
	self.KP = 0.02
	self.KI = 0
	self.KD = 0
	self.previous_error = 0.0
	self.integral = 0.0
	self.last_time = time.time()


    def execute(self):
	if self.index < len(self.positions):
		current_time = time.time()
		dt = current_time - self.last_time
		x = self.sensor.getPose3D().x
		y = self.sensor.getPose3D().y
		q0 = self.sensor.getPose3D().q0
		q1 = self.sensor.getPose3D().q1
		q2 = self.sensor.getPose3D().q2
		q3 = self.sensor.getPose3D().q3
	
		angle1 = self.angle(x, y, self.positions[self.index][0], self.positions[self.index][1])
		angle2 = self.quaternions2angle(q0,q1,q2,q3)
		error = angle1 - angle2
		if (angle2 > angle1 and (angle2 - angle1) > ((angle1 + 360) - angle2)) or  (angle2 < angle1 and (angle1 - angle2) > ((angle2 + 360) - angle1)):
			error *= -1
		
		self.integral += error * dt
		derivative = (error - self.previous_error) / dt
		rotation = self.KP * error + self.KI * self.integral + self.KD * derivative
		self.previous_error = error
		self.last_time = current_time
	
		distance = self.hypotenuse(x, y, self.positions[self.index][0], self.positions[self.index][1])
		print "x1:%.2f y1:%.2f x2:%.2f y2:%.2f d:%.2f a1:%.2f a2:%.2f r:%f" % (x, y, self.positions[self.index][0], self.positions[self.index][1], distance, angle1, angle2, rotation)

		if distance < 1:
			self.index += 1


		self.sensor.sendCMDVel(0, self.speed, 0, rotation, 0, 0)

		
	else:
		self.sensor.sendCMDVel(0, 0, 0, 0, 0, 0)


    def hypotenuse(self, x1, y1, x2, y2):
	return pow(pow(x2 - x1, 2) + pow(y2 - y1, 2), 0.5)

    def angle(self, x1, y1, x2, y2):
	return self.radians2degrees(atan2(y2 - y1, x2 - x1))
	
    def quaternions2angle(self,qw,qx,qy,qz):
        rotateZa0=2.0*(qx*qy + qw*qz)
        rotateZa1=qw*qw + qx*qx - qy*qy - qz*qz
        rotateZ=0.0
        if(rotateZa0 != 0.0 and rotateZa1 != 0.0):
            rotateZ=atan2(rotateZa0,rotateZa1)
        return self.radians2degrees(rotateZ)

    def radians2degrees(self, angle):
	if angle < 0:
		angle = 2 * pi + angle
	return angle * 360 / (2 * pi)




